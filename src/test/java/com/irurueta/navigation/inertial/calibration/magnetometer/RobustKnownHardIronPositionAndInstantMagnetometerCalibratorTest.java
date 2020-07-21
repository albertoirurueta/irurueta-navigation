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
import static org.junit.Assert.assertSame;

public class RobustKnownHardIronPositionAndInstantMagnetometerCalibratorTest implements
        RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate4() {
        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        magneticModel, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(magneticModel, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testCreate6() {
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(initialHardIron, initialMm,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate9() {
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
    }

    @Test
    public void testCreate10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreate17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreate21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate44() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, true,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate45() {
        final double[] qualityScores = new double[10];
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, magneticModel,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, magneticModel,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate46() {
        final double[] qualityScores = new double[10];
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate47() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate48() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator
                .create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate49() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, true,
                        initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements, initialHardIron,
                        initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                qualityScores, position, measurements, true,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        magneticModel);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final double[] initialHardIron = generateHardIron();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreateWithDefaultMethod9() {
        final NEDPosition position = new NEDPosition();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
    }

    @Test
    public void testCreateWithDefaultMethod10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod16() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
    }

    @Test
    public void testCreateWithDefaultMethod17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
                0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod18() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod20() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
    }

    @Test
    public void testCreateWithDefaultMethod21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron,
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
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

        final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
                        position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronAsMatrix(),
                initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator,
            final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator,
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
