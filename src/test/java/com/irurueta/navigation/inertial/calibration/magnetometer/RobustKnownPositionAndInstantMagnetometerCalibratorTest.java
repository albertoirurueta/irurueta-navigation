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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RobustKnownPositionAndInstantMagnetometerCalibratorTest implements
        RobustKnownPositionAndInstantMagnetometerCalibratorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate4() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        magneticModel, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testCreate6() {
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate9() {
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
    }

    @Test
    public void testCreate10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate12() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate13() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate14() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate16() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate18() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate20() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate22() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate23() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate24() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate25() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate26() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    public void testCreate27() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate28() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate29() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate30() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate31() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate32() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate33() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate34() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate35() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate36() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate37() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate38() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate39() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate40() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate41() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate42() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate43() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate44() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate45() {
        final double[] qualityScores = new double[10];
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, magneticModel,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate46() {
        final double[] qualityScores = new double[10];
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate47() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate48() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate49() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate50() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate51() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate52() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate53() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate54() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate55() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate56() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate57() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate58() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate59() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate60() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate61() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate62() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate63() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate64() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate65() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate66() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate67() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate68() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate69() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate70() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate71() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate72() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate73() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate74() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate75() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate76() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate77() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate78() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate79() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate80() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate81() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate82() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        magneticModel);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod9() {
        final NEDPosition position = new NEDPosition();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
    }

    @Test
    public void testCreateWithDefaultMethod10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod12() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod13() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod14() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod16() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod18() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod20() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod22() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod23() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod24() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod25() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod26() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    public void testCreateWithDefaultMethod27() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod28() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod29() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod30() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod31() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod32() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod33() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod34() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod35() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod36() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod37() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod38() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod39() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod40() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod41() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod42() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator,
            final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator,
            final float progress) {
    }

    private double[] generateHardIron() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());

        final double[] result = new double[3];
        randomizer.fill(result);

        return result;
    }

    private Matrix generateHardIronMatrix() {
        return Matrix.newFromArray(generateHardIron());
    }

    private Matrix generateMm() {
        try {
            return Matrix.createWithUniformRandomValues(
                    3, 3, -1.0, 1.0);
        } catch (WrongSizeException ignore) {
            return null;
        }
    }
}
