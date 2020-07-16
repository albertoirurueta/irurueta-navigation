package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.assertFalse;

public class LMedSRobustKnownPositionAndInstantMagnetometerCalibratorTest implements
        RobustKnownPositionAndInstantMagnetometerCalibratorListener {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-2;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 4;

    private static final double THRESHOLD = 1e-9;

    private static final double OUTLIER_ERROR_FACTOR = 1000.0;

    private static final int TIMES = 100;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
                final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        measurements);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
            final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        magneticModel);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        hardIron);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        bm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        bm, mm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getNedPosition());
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    bm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    bm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, hardIron);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, hardIron, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true, hardIron);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true,
                        hardIron, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, bm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, bm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true, bm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true, bm,
                        this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, bm, mm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, bm,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, bm,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, bm, mm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, bm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, bm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true,
                        bm, mm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    bm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    bm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        nedPosition, measurements, true,
                        bm, mm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertEquals(ecefPosition, ecefPosition1);
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    bm, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    nedPosition, measurements, true,
                    bm, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, hardIron);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, hardIron, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true, hardIron);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true,
                        hardIron, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, bm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, bm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true, bm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true, bm,
                        this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, new Matrix(3, 3));
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, bm, mm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, bm,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, bm,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, bm, mm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, bm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, bm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true,
                        bm, mm);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    bm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    bm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor42() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                        ecefPosition, measurements, true,
                        bm, mm, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getInitialHardIronX(), bmx, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), bmy, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), bmz, 0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition1 = new ECEFPosition();
        assertTrue(calibrator.getEcefPosition(ecefPosition1));
        assertTrue(ecefPosition1.equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertNotNull(calibrator.getYear());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(), 13);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    bm, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    ecefPosition, measurements, true,
                    bm, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);

        // set new value
        calibrator.setStopThreshold(THRESHOLD);

        // check
        assertEquals(calibrator.getStopThreshold(), THRESHOLD, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialHardIronX() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];

        calibrator.setInitialHardIronX(hardIronX);

        // check
        assertEquals(calibrator.getInitialHardIronX(), hardIronX, 0.0);
    }

    @Test
    public void testGetSetInitialHardIronY() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronY = mb[1];

        calibrator.setInitialHardIronY(hardIronY);

        // check
        assertEquals(calibrator.getInitialHardIronY(), hardIronY, 0.0);
    }

    @Test
    public void testGetSetInitialHardIronZ() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronZ = mb[2];

        calibrator.setInitialHardIronZ(hardIronZ);

        // check
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testSetInitialHardIron() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        calibrator.setInitialHardIron(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(calibrator.getInitialHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testGetSetInitialSx() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sy = mm.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sz = mm.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxz = mm.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myx = mm.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myz = mm.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzx = mm.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(mxy, mxz, myx,
                myz, mzx, mzy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactorsAndCrossCouplingErrors()
            throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testGetInitialHardIronAsArray() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] result1 = new double[3];
        calibrator.getInitialHardIron(result1);
        assertArrayEquals(result1, new double[3], 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        calibrator.setInitialHardIron(bm);

        // check
        assertArrayEquals(calibrator.getInitialHardIron(), bm, 0.0);
        final double[] result2 = new double[3];
        calibrator.getInitialHardIron(result2);
        assertArrayEquals(result2, bm, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetInitialHardIronAsMatrix() throws LockedException,
            WrongSizeException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix result1 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(result1);
        assertEquals(result1, new Matrix(3, 1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        final Matrix b = Matrix.newFromArray(bm);
        calibrator.setInitialHardIron(b);

        // check
        assertEquals(calibrator.getInitialHardIronAsMatrix(), b);
        final Matrix result2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(result2);
        assertEquals(result2, b);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialHardIronAsMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialHardIronAsMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialMm() throws WrongSizeException,
            LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMm(), new Matrix(3, 3));
        final Matrix result1 = new Matrix(3, 3);
        calibrator.getInitialMm(result1);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        calibrator.setInitialMm(mm);

        // check
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix result2 = new Matrix(3, 3);
        calibrator.getInitialMm(result2);
        assertEquals(mm, result2);
    }

    @Test
    public void testGetSetNedPosition() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        calibrator.setPosition(nedPosition);

        // check
        assertSame(calibrator.getNedPosition(), nedPosition);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetEcefPosition() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getEcefPosition());
        assertFalse(calibrator.getEcefPosition(null));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition nedPosition = createPosition(randomizer);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        calibrator.setPosition(ecefPosition);

        // check
        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        calibrator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition2.equals(ecefPosition,
                LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetYear() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNotNull(calibrator.getYear());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        long timestamp = createTimestamp(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        final double year = WMMEarthMagneticFluxDensityEstimator
                .convertTime(calendar);

        calibrator.setYear(year);

        // check
        assertEquals(calibrator.getYear(), year, 0.0);
    }

    @Test
    public void testSetTime1() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNotNull(calibrator.getYear());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        long timestamp = createTimestamp(randomizer);
        final Date date = new Date(timestamp);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        final double year = WMMEarthMagneticFluxDensityEstimator
                .convertTime(calendar);

        calibrator.setTime(date);

        // check
        assertEquals(calibrator.getYear(), year, 0.0);
    }

    @Test
    public void testSetTime2() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNotNull(calibrator.getYear());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        long timestamp = createTimestamp(randomizer);
        final Date date = new Date(timestamp);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        final double year = WMMEarthMagneticFluxDensityEstimator
                .convertTime(calendar);

        calibrator.setTime(date.getTime());

        // check
        assertEquals(calibrator.getYear(), year, 0.0);
    }

    @Test
    public void testSetTime3() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNotNull(calibrator.getYear());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        long timestamp = createTimestamp(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        final double year = WMMEarthMagneticFluxDensityEstimator
                .convertTime(calendar);

        calibrator.setTime(calendar);

        // check
        assertEquals(calibrator.getYear(), year, 0.0);
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetMinimumRequiredMeasurements() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 13);
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsReady() throws LockedException, IOException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // initially there are no measurements
        assertFalse(calibrator.isReady());
        assertNull(calibrator.getMeasurements());

        // set not enough measurements
        final List<StandardDeviationBodyMagneticFluxDensity> measurements1 =
                Collections.emptyList();
        calibrator.setMeasurements(measurements1);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final NEDPosition position = createPosition(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIronGeneral();
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements2 =
                generateMeasures(hardIron, softIron,
                        calibrator.getMinimumRequiredMeasurements(),
                        wmmEstimator, randomizer,
                        position, timestamp);
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        // set position
        calibrator.setPosition(position);

        // check
        assertTrue(calibrator.isReady());

        // unset year
        calibrator.setYear(null);

        // check
        assertFalse(calibrator.isReady());
    }

    @Test
    public void testGetSetMagneticModel() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getProgressDelta(), 0.05f, 0.0);

        // set new value
        calibrator.setProgressDelta(0.01f);

        // check
        assertEquals(calibrator.getProgressDelta(), 0.01f, 0.0);

        // force IllegalArgumentException
        try {
            calibrator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getConfidence(), 0.99, 0.0);

        // set new value
        calibrator.setConfidence(0.5);

        // check
        assertEquals(calibrator.getConfidence(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getMaxIterations(), 5000);

        // set new value
        calibrator.setMaxIterations(100);

        assertEquals(calibrator.getMaxIterations(), 100);

        // Force IllegalArgumentException
        try {
            calibrator.setMaxIterations(0);
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                LMedSRobustKnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);

        // set new value
        calibrator.setPreliminarySubsetSize(11);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 11);

        // Force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(9);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateGeneralNoNoiseInlier()
            throws IOException, LockedException, WrongSizeException,
            CalibrationException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            null, position, timestamp, cnb);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                            position, measurements, false,
                            bm, mm, this);
            calibrator.setTime(timestamp);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisNoNoiseInlier()
            throws IOException, LockedException, CalibrationException,
            NotReadyException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            null, position, timestamp, cnb);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                            position, measurements, true,
                            bm, mm, this);
            calibrator.setTime(timestamp);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralWithInlierNoise()
            throws IOException, LockedException, CalibrationException,
            NotReadyException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            outlierNoiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            inlierNoiseRandomizer, position, timestamp, cnb);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                            position, measurements, false,
                            bm, mm, this);
            calibrator.setTime(timestamp);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisWithInlierNoise()
            throws IOException, LockedException, CalibrationException,
            NotReadyException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            outlierNoiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            inlierNoiseRandomizer, position, timestamp, cnb);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                            position, measurements, true,
                            bm, mm, this);
            calibrator.setTime(timestamp);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNoRefinement()
            throws IOException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator,
                            null, position, timestamp, cnb);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                            position, measurements, false,
                            bm, mm, this);
            calibrator.setTime(timestamp);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setResultRefined(false);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
    
    @Override
    public void onCalibrateStart(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownPositionAndInstantMagnetometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownPositionAndInstantMagnetometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator,
            final int iteration) {
        checkLocked((LMedSRobustKnownPositionAndInstantMagnetometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator,
            final float progress) {
        checkLocked((LMedSRobustKnownPositionAndInstantMagnetometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(
            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        try {
            calibrator.setStopThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIron(
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMxy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMxz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMyx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMyz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMzx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMzy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialScalingFactors(
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIron((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIron((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMm(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPosition((NEDPosition) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPosition((ECEFPosition) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setYear(2020.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTime(0L);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTime((Date) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTime((GregorianCalendar) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMeasurements(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCommonAxisUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMagneticModel(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPreliminarySubsetSize(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix hardIron, final Matrix mm,
            final LMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator)
            throws WrongSizeException {

        final double[] estimatedHardIron = calibrator.getEstimatedHardIron();
        assertArrayEquals(hardIron.getBuffer(), estimatedHardIron, 0.0);

        final double[] estimatedHardIron2 = new double[3];
        assertTrue(calibrator.getEstimatedHardIron(estimatedHardIron2));
        assertArrayEquals(estimatedHardIron, estimatedHardIron2, 0.0);

        final Matrix hardIron2 = new Matrix(3, 1);
        assertTrue(calibrator.getEstimatedHardIronAsMatrix(hardIron2));

        assertEquals(hardIron, hardIron2);

        assertEquals(hardIron.getElementAtIndex(0),
                calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(hardIron.getElementAtIndex(1),
                calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(hardIron.getElementAtIndex(2),
                calibrator.getEstimatedHardIronZ(), 0.0);

        assertEquals(mm.getElementAt(0, 0), calibrator.getEstimatedSx(),
                0.0);
        assertEquals(mm.getElementAt(1, 1), calibrator.getEstimatedSy(),
                0.0);
        assertEquals(mm.getElementAt(2, 2), calibrator.getEstimatedSz(),
                0.0);
        assertEquals(mm.getElementAt(0, 1), calibrator.getEstimatedMxy(),
                0.0);
        assertEquals(mm.getElementAt(0, 2), calibrator.getEstimatedMxz(),
                0.0);
        assertEquals(mm.getElementAt(1, 0), calibrator.getEstimatedMyx(),
                0.0);
        assertEquals(mm.getElementAt(1, 2), calibrator.getEstimatedMyz(),
                0.0);
        assertEquals(mm.getElementAt(2, 0), calibrator.getEstimatedMzx(),
                0.0);
        assertEquals(mm.getElementAt(2, 1), calibrator.getEstimatedMzy(),
                0.0);
    }

    private static List<StandardDeviationBodyMagneticFluxDensity> generateMeasures(
            final double[] hardIron, final Matrix softIron,
            final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final NEDPosition position,
            final Date timestamp) {

        final List<StandardDeviationBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < numberOfMeasurements; i++) {
            final CoordinateTransformation cnb = generateBodyC(randomizer);
            result.add(generateMeasure(hardIron, softIron, wmmEstimator,
                    null, position, timestamp, cnb));
        }
        return result;
    }

    private static StandardDeviationBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer,
            final NEDPosition position,
            final Date timestamp,
            final CoordinateTransformation cnb) {

        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity truthMagnetic =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final BodyMagneticFluxDensity measuredMagnetic =
                generateMeasuredMagneticFluxDensity(truthMagnetic,
                        hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz()
                    + noiseRandomizer.nextDouble());
        }

        final double std = noiseRandomizer != null ?
                noiseRandomizer.getStandardDeviation() :
                MAGNETOMETER_NOISE_STD;
        return new StandardDeviationBodyMagneticFluxDensity(
                measuredMagnetic, std);
    }

    private static CoordinateTransformation generateBodyC(
            final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron,
            final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron,
                softIron);
    }

    private static double[] generateHardIron(
            final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static Matrix generateSoftIronCommonAxis() {
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        for (int col = 0; col < mm.getColumns(); col++) {
            for (int row = 0; row < mm.getRows(); row++) {
                if (row > col) {
                    mm.setElementAt(row, col, 0.0);
                }
            }
        }
        return mm;
    }

    private static NEDPosition createPosition(
            final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
