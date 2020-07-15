package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;
import static org.junit.Assert.assertSame;

public class RobustKnownHardIronAndFrameMagnetometerCalibratorTest implements
        RobustKnownHardIronAndFrameMagnetometerCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // MSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate5() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate6() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate9() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate10() {
        final double[] qualityScores = new double[4];

        // MSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate11() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate12() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate13() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate14() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, true,
                        this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, true,
                this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate15() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate16() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        qualityScores, measurements,
                        true,
                        this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(),
                qualityScores);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                qualityScores, measurements,
                true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(),
                qualityScores);
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        true, this);
        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(
                        measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }
    
    @Override
    public void onCalibrateStart(RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        
    }

    @Override
    public void onCalibrateEnd(RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {

    }

    @Override
    public void onCalibrateNextIteration(RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, int iteration) {

    }

    @Override
    public void onCalibrateProgressChange(RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, float progress) {

    }
}
