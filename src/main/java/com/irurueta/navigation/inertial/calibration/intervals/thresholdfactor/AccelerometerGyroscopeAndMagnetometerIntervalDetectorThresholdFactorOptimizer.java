/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.QualityScoredAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.UnknownBiasAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.OrderedBodyKinematicsSequenceGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QualityScoredGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.UnknownBiasGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.QualityScoredMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.UnknownHardIronMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;

import java.util.ArrayList;
import java.util.List;

/**
 * Optimizes threshold factor for interval detection of accelerometer + gyroscope
 * data based on results of calibration.
 * Implementations of this class will attempt to find best threshold factor
 * between provided range of values.
 * Only accelerometer calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link AccelerometerNonLinearCalibrator} and must support
 * {@link AccelerometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_KINEMATICS}).
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 * Only magnetometer calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 */
public abstract class AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer extends
        IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematicsAndMagneticFluxDensity,
                AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> implements
        AccelerometerNoiseRootPsdSource, GyroscopeNoiseRootPsdSource {

    /**
     * Default minimum threshold factor.
     */
    public static final double DEFAULT_MIN_THRESHOLD_FACTOR = 2.0;

    /**
     * Default maximum threshold factor.
     */
    public static final double DEFAULT_MAX_THRESHOLD_FACTOR = 10.0;

    /**
     * Minimum threshold factor.
     */
    protected double mMinThresholdFactor = DEFAULT_MIN_THRESHOLD_FACTOR;

    /**
     * Maximum threshold factor.
     */
    protected double mMaxThresholdFactor = DEFAULT_MAX_THRESHOLD_FACTOR;

    /**
     * Accelerometer calibrator.
     */
    private AccelerometerNonLinearCalibrator mAccelerometerCalibrator;

    /**
     * Gyroscope calibrator.
     */
    private GyroscopeNonLinearCalibrator mGyroscopeCalibrator;

    /**
     * Magnetometer calibrator.
     */
    private MagnetometerNonLinearCalibrator mMagnetometerCalibrator;

    /**
     * A measurement generator for accelerometer, gyroscope and magnetometer calibrators.
     */
    private AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator mGenerator;

    /**
     * Generated measurements to be used for accelerometer calibration.
     */
    private List<StandardDeviationBodyKinematics> mAccelerometerMeasurements;

    /**
     * Generated sequences to be used for gyroscope calibration.
     */
    private List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mGyroscopeSequences;

    /**
     * Generated measurements to be used for magnetometer calibration.
     */
    private List<StandardDeviationBodyMagneticFluxDensity> mMagnetometerMeasurements;

    /**
     * Mapper to convert {@link StandardDeviationBodyKinematics} measurements into
     * quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyKinematics> mAccelerometerQualityScoreMapper =
            new DefaultAccelerometerQualityScoreMapper();

    /**
     * Mapper to convert {@link BodyKinematicsSequence} sequences of {@link StandardDeviationTimedBodyKinematics}
     * into quality scores.
     */
    private QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
            mGyroscopeQualityScoreMapper = new DefaultGyroscopeQualityScoreMapper();

    /**
     * Mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements
     * into quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> mMagnetometerQualityScoreMapper =
            new DefaultMagnetometerQualityScoreMapper();

    /**
     * Rule to convert accelerometer, gyroscope and magnetometer MSE
     * values into a single global MSE value.
     */
    private AccelerometerGyroscopeAndMagnetometerMseRule mMseRule =
            new DefaultAccelerometerGyroscopeAndMagnetometerMseRule();

    /**
     * Estimated norm of gyroscope noise root PSD (Power Spectral Density)
     * expressed as (rad * s^-0.5).
     */
    private double mAngularSpeedNoiseRootPsd;

    /**
     * Accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found expressed in
     * meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     */
    private double mBaseNoiseLevel;

    /**
     * Threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for the best calibration solution that
     * has been found.
     */
    private double mThreshold;

    /**
     * Estimated covariance matrix for estimated accelerometer parameters.
     */
    private Matrix mEstimatedAccelerometerCovariance;

    /**
     * Estimated accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedAccelerometerMa;

    /**
     * Estimated accelerometer biases for each IMU axis expressed in meter per squared
     * second (m/s^2).
     */
    private double[] mEstimatedAccelerometerBiases;

    /**
     * Estimated covariance matrix for estimated gyroscope parameters.
     */
    private Matrix mEstimatedGyroscopeCovariance;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
     */
    private double[] mEstimatedGyroscopeBiases;

    /**
     * Estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedGyroscopeMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix mEstimatedGyroscopeGg;

    /**
     * Estimated covariance matrix for estimated magnetometer parameters.
     */
    private Matrix mEstimatedMagnetometerCovariance;

    /**
     * Estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMagnetometerMm;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] mEstimatedMagnetometerHardIron;

    /**
     * Constructor.
     */
    public AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer() {
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrator an accelerometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @param gyroscopeCalibrator     a gyroscope calibrator to be used to optimize
     *                                its Mean Square Error (MSE).
     * @param magnetometerCalibrator  a magnetometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        try {
            setAccelerometerCalibrator(accelerometerCalibrator);
            setGyroscopeCalibrator(gyroscopeCalibrator);
            setMagnetometerCalibrator(magnetometerCalibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource              instance in charge of retrieving data for this optimizer.
     * @param accelerometerCalibrator an accelerometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @param gyroscopeCalibrator     a gyroscope calibrator to be used to optimize
     *                                its Mean Square Error (MSE).
     * @param magnetometerCalibrator  a magnetometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        super(dataSource);
        try {
            setAccelerometerCalibrator(accelerometerCalibrator);
            setGyroscopeCalibrator(gyroscopeCalibrator);
            setMagnetometerCalibrator(magnetometerCalibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Gets provided accelerometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return accelerometer calibrator to be used to optimize its MSE.
     */
    public AccelerometerNonLinearCalibrator getAccelerometerCalibrator() {
        return mAccelerometerCalibrator;
    }

    /**
     * Sets accelerometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @param accelerometerCalibrator accelerometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements.
     */
    public void setAccelerometerCalibrator(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (accelerometerCalibrator != null && accelerometerCalibrator.getMeasurementType() !=
                AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS) {
            throw new IllegalArgumentException();
        }

        mAccelerometerCalibrator = accelerometerCalibrator;
    }

    /**
     * Gets provided gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return gyroscope calibrator to be used to optimize its MSE.
     */
    public GyroscopeNonLinearCalibrator getGyroscopeCalibrator() {
        return mGyroscopeCalibrator;
    }

    /**
     * Sets gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @param gyroscopeCalibrator gyroscope calibrator to be use dto optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics}.
     */
    public void setGyroscopeCalibrator(
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (gyroscopeCalibrator != null && gyroscopeCalibrator.getMeasurementOrSequenceType() !=
                GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            throw new IllegalArgumentException();
        }

        mGyroscopeCalibrator = gyroscopeCalibrator;
    }

    /**
     * Gets provided magnetometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return magnetometer calibrator to be used to optimize its MSE.
     */
    public MagnetometerNonLinearCalibrator getMagnetometerCalibrator() {
        return mMagnetometerCalibrator;
    }

    /**
     * Sets magnetometer calibrator to be used to optimize its Mean Square Error.
     *
     * @param magnetometerCalibrator magnetometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public void setMagnetometerCalibrator(
            final MagnetometerNonLinearCalibrator magnetometerCalibrator)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (magnetometerCalibrator != null && magnetometerCalibrator.getMeasurementType() !=
                MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY) {
            throw new IllegalArgumentException();
        }

        mMagnetometerCalibrator = magnetometerCalibrator;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyKinematics} accelerometer measurements
     * into quality scores.
     *
     * @return mapper to convert accelerometer measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyKinematics> getAccelerometerQualityScoreMapper() {
        return mAccelerometerQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyKinematics} accelerometer measurements
     * into quality scores.
     *
     * @param accelerometerQualityScoreMapper mapper to convert accelerometer measurements into
     *                                        quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setAccelerometerQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyKinematics> accelerometerQualityScoreMapper)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerQualityScoreMapper = accelerometerQualityScoreMapper;
    }

    /**
     * Gets mapper to convert {@link BodyKinematicsSequence} gyroscope sequences of
     * {@link StandardDeviationTimedBodyKinematics} into quality scores.
     *
     * @return mapper to convert gyroscope sequences into quality scores.
     */
    public QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
    getGyroscopeQualityScoreMapper() {
        return mGyroscopeQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link BodyKinematicsSequence} gyroscope sequences of
     * {@link StandardDeviationTimedBodyKinematics} into quality scores.
     *
     * @param gyroscopeQualityScoreMapper mapper to convert gyroscope sequences
     *                                    into quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setGyroscopeQualityScoreMapper(
            final QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
                    gyroscopeQualityScoreMapper)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGyroscopeQualityScoreMapper = gyroscopeQualityScoreMapper;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} magnetometer
     * measurements into quality scores.
     *
     * @return mapper to convert magnetometer measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> getMagnetometerQualityScoreMapper() {
        return mMagnetometerQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} magnetometer
     * measurements into quality scores.
     *
     * @param magnetometerQualityScoreMapper mapper to convert magnetometer measurements into
     *                                       quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setMagnetometerQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> magnetometerQualityScoreMapper)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mMagnetometerQualityScoreMapper = magnetometerQualityScoreMapper;
    }

    /**
     * Gets rule to convert accelerometer, gyroscope and magnetometer
     * MSE values into a single global MSE value.
     *
     * @return rule to convert accelerometer, gyroscope and
     * magnetometer MSE values into a single global MSE value.
     */
    public AccelerometerGyroscopeAndMagnetometerMseRule getMseRule() {
        return mMseRule;
    }

    /**
     * Sets rule to convert accelerometer, gyroscope and magnetometer
     * MSE values into a single global MSE value.
     *
     * @param mseRule rule to convert accelerometer, gyroscope and
     *                magnetometer MSE values into a single global
     *                MSE value.
     * @throws LockedException if optimizer is already running.
     */
    public void setMseRule(
            final AccelerometerGyroscopeAndMagnetometerMseRule mseRule)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mMseRule = mseRule;
    }

    /**
     * Gets minimum threshold factor.
     *
     * @return minimum threshold factor.
     */
    public double getMinThresholdFactor() {
        return mMinThresholdFactor;
    }

    /**
     * Gets maximum threshold factor.
     *
     * @return maximum threshold factor.
     */
    public double getMaxThresholdFactor() {
        return mMaxThresholdFactor;
    }

    /**
     * Sets range of threshold factor values to obtain an optimized
     * threshold factor value.
     *
     * @param minThresholdFactor minimum threshold.
     * @param maxThresholdFactor maximum threshold.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if either minimum or maximum values are
     *                                  negative or if minimum value is larger
     *                                  than maximum one.
     */
    public void setThresholdFactorRange(
            final double minThresholdFactor, final double maxThresholdFactor)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (minThresholdFactor < 0.0 || maxThresholdFactor < 0.0
                || minThresholdFactor >= maxThresholdFactor) {
            throw new IllegalArgumentException();
        }

        mMinThresholdFactor = minThresholdFactor;
        mMaxThresholdFactor = maxThresholdFactor;
    }

    /**
     * Indicates whether this optimizer is ready to start optimization.
     *
     * @return true if this optimizer is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mAccelerometerCalibrator != null
                && mGyroscopeCalibrator != null
                && mMagnetometerCalibrator != null
                && mAccelerometerQualityScoreMapper != null
                && mGyroscopeQualityScoreMapper != null
                && mMagnetometerQualityScoreMapper != null
                && mMseRule != null;
    }

    /**
     * Gets time interval between input measurements provided to the
     * {@link #getDataSource()} expressed in seconds (s).
     *
     * @return time interval between input measurements.
     */
    public double getTimeInterval() {
        return mGenerator.getTimeInterval();
    }

    /**
     * Sets time interval between input measurements provided to the
     * {@link #getDataSource()} expressed in seconds (s).
     *
     * @param timeInterval time interval between input measurements.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @return time interval between input measurements.
     */
    public Time getTimeIntervalAsTime() {
        return mGenerator.getTimeIntervalAsTime();
    }

    /**
     * Gets time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        mGenerator.getTimeIntervalAsTime(result);
    }

    /**
     * Sets time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @param timeInterval time interval between input measurements.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setTimeInterval(timeInterval);
    }

    /**
     * Gets minimum number of input measurements provided to the
     * {@link #getDataSource()} required in a static interval to be taken
     * into account.
     * Smaller static intervals will be discarded.
     *
     * @return minimum number of input measurements required in a static interval
     * to be taken into account.
     */
    public int getMinStaticSamples() {
        return mGenerator.getMinStaticSamples();
    }

    /**
     * Sets minimum number of input measurements provided to the
     * {@link #getDataSource()} required in a static interval to be taken
     * into account.
     * Smaller static intervals will be discarded.
     *
     * @param minStaticSamples minimum number of input measurements required in
     *                         a static interval to be taken into account.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMinStaticSamples(final int minStaticSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setMinStaticSamples(minStaticSamples);
    }

    /**
     * Gets maximum number of input measurements provided to the
     * {@link #getDataSource()} allowed in dynamic intervals.
     *
     * @return maximum number of input measurements allowed in dynamic intervals.
     */
    public int getMaxDynamicSamples() {
        return mGenerator.getMaxDynamicSamples();
    }

    /**
     * Sets maximum number of input measurements provided to the
     * {@link #getDataSource()} allowed in dynamic intervals.
     *
     * @param maxDynamicSamples maximum number of input measurements allowed in
     *                          dynamic intervals.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMaxDynamicSamples(final int maxDynamicSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setMaxDynamicSamples(maxDynamicSamples);
    }

    /**
     * Gets length of number of input measurements provided to the
     * {@link #getDataSource()} to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of input measurements to keep within the window.
     */
    public int getWindowSize() {
        return mGenerator.getWindowSize();
    }

    /**
     * Sets length of number of input measurements provided to the
     * {@link #getDataSource()} to keep within the window being processed
     * to determine instantaneous noise level.
     * Window size must always be larger than allowed minimum value, which is 2
     * and must have an odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setWindowSize(windowSize);
    }

    /**
     * Gets number of input measurements provided to the
     * {@link #getDataSource()} to be processed initially while keeping he sensor
     * static in order to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return mGenerator.getInitialStaticSamples();
    }

    /**
     * Sets number of input parameters provided to the {@link #getDataSource()}
     * to be processed initially while keeping the sensor static in order to
     * find the base noise level when device is static.
     *
     * @param initialStaticSamples number of samples ot be processed initially.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link TriadStaticIntervalDetector#MINIMUM_INITIAL_STATIC_SAMPLES}.
     */
    public void setInitialStaticSamples(final int initialStaticSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setInitialStaticSamples(initialStaticSamples);
    }

    /**
     * Gets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @return factor to determine that a sudden movement has occurred.
     */
    public double getInstantaneousNoiseLevelFactor() {
        return mGenerator.getInstantaneousNoiseLevelFactor();
    }

    /**
     * Sets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @param instantaneousNoiseLevelFactor factor to determine that a sudden
     *                                      movement has occurred during
     *                                      initialization.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setInstantaneousNoiseLevelFactor(
            final double instantaneousNoiseLevelFactor) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setInstantaneousNoiseLevelFactor(
                instantaneousNoiseLevelFactor);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @return overall absolute threshold to determine whether there has been
     * excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return mGenerator.getBaseNoiseLevelAbsoluteThreshold();
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(
            final double baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @return overall asolute threshold to determine whether there has been
     * excessive motion.
     */
    public Acceleration getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return mGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            final Acceleration result) {
        mGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result);
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(
            final Acceleration baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found expressed in
     * meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return accelerometer base noise level of best solution that has been
     * found.
     */
    public double getAccelerometerBaseNoiseLevel() {
        return mBaseNoiseLevel;
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found.
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return accelerometer base noise level of best solution that has been
     * found.
     */
    public Acceleration getAccelerometerBaseNoiseLevelAsMeasurement() {
        return createMeasurement(mBaseNoiseLevel, getDefaultUnit());
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found.
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(
            final Acceleration result) {
        result.setValue(mBaseNoiseLevel);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return mAngularSpeedNoiseRootPsd * mAngularSpeedNoiseRootPsd;
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return mAngularSpeedNoiseRootPsd;
    }

    /**
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3).
     *
     * @return accelerometer base noise level PSD.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return mBaseNoiseLevel * mBaseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return mBaseNoiseLevel * Math.sqrt(getTimeInterval());
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for the best calibration solution that
     * has been found.
     *
     * @return threshold to determine static/dynamic period changes for best
     * solution.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Gets threshold to determine static/dynamic period changes for the best
     * calibration solution that has been found.
     *
     * @return threshold to determine static/dynamic period changes for best
     * solution.
     */
    public Acceleration getThresholdAsMeasurement() {
        return createMeasurement(mThreshold, getDefaultUnit());
    }

    /**
     * Get threshold to determine static/dynamic period changes for the best
     * calibration solution that has been found.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final Acceleration result) {
        result.setValue(mThreshold);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of accelerometer bias or null
     * if not available.
     */
    public Double getEstimatedAccelerometerBiasStandardDeviationNorm() {
        return mEstimatedAccelerometerCovariance != null ?
                Math.sqrt(getEstimatedAccelerometerBiasFxVariance()
                        + getEstimatedAccelerometerBiasFyVariance()
                        + getEstimatedAccelerometerBiasFzVariance()) :
                null;
    }

    /**
     * Gets variance of estimated x coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated x coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFxVariance() {
        return mEstimatedAccelerometerCovariance != null
                ? mEstimatedAccelerometerCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets variance of estimated y coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated y coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFyVariance() {
        return mEstimatedAccelerometerCovariance != null
                ? mEstimatedAccelerometerCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets variance of estimated z coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated z coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFzVariance() {
        return mEstimatedAccelerometerCovariance != null
                ? mEstimatedAccelerometerCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    public double[] getEstimatedAccelerometerBiases() {
        return mEstimatedAccelerometerBiases;
    }

    /**
     * Gets estimated accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated accelerometer scale factors and cross coupling errors, or null
     * if not available.
     */
    public Matrix getEstimatedAccelerometerMa() {
        return mEstimatedAccelerometerMa;
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias expressed in
     * radians per second (rad/s).
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    public Double getEstimatedGyroscopeBiasStandardDeviationNorm() {
        return mEstimatedGyroscopeCovariance != null ?
                Math.sqrt(getEstimatedGyroscopeBiasXVariance()
                        + getEstimatedGyroscopeBiasYVariance()
                        + getEstimatedGyroscopeBiasZVariance()) :
                null;
    }

    /**
     * Gets variance of estimated x coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated x coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasXVariance() {
        return mEstimatedGyroscopeCovariance != null
                ? mEstimatedGyroscopeCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets variance of estimated y coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated y coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasYVariance() {
        return mEstimatedGyroscopeCovariance != null
                ? mEstimatedGyroscopeCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets variance of estimated z coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated z coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasZVariance() {
        return mEstimatedGyroscopeCovariance != null
                ? mEstimatedGyroscopeCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    public double[] getEstimatedGyroscopeBiases() {
        return mEstimatedGyroscopeBiases;
    }

    /**
     * Gets estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated gyroscope scale factors and cross coupling errors.
     */
    public Matrix getEstimatedGyroscopeMg() {
        return mEstimatedGyroscopeMg;
    }

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    public Matrix getEstimatedGyroscopeGg() {
        return mEstimatedGyroscopeGg;
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    public double[] getEstimatedMagnetometerHardIron() {
        return mEstimatedMagnetometerHardIron;
    }

    /**
     * Gets estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated magnetometer soft-iron scale factors and cross coupling errors,
     * or null if not available.
     */
    public Matrix getEstimatedMagnetometerMm() {
        return mEstimatedMagnetometerMm;
    }

    /**
     * Evaluates calibration Mean Square Error (MSE) for provided threshold factor.
     *
     * @param thresholdFactor threshold factor to be used for interval detection
     *                        and measurements generation to be used for
     *                        calibration.
     * @return calibration MSE.
     * @throws LockedException                                   if generator is busy.
     * @throws CalibrationException                              if calibration fails.
     * @throws NotReadyException                                 if calibrator is not ready.
     * @throws IntervalDetectorThresholdFactorOptimizerException interval detection failed.
     */
    protected double evaluateForThresholdFactor(final double thresholdFactor)
            throws LockedException, CalibrationException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException {
        if (mAccelerometerMeasurements == null) {
            mAccelerometerMeasurements = new ArrayList<>();
        } else {
            mAccelerometerMeasurements.clear();
        }

        if (mGyroscopeSequences == null) {
            mGyroscopeSequences = new ArrayList<>();
        } else {
            mGyroscopeSequences.clear();
        }

        if (mMagnetometerMeasurements == null) {
            mMagnetometerMeasurements = new ArrayList<>();
        } else {
            mMagnetometerMeasurements.clear();
        }

        mGenerator.reset();
        mGenerator.setThresholdFactor(thresholdFactor);

        int count = mDataSource.count();
        boolean failed = false;
        for (int i = 0; i < count; i++) {
            final TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics = mDataSource.getAt(i);
            if (!mGenerator.process(timedBodyKinematics)) {
                failed = true;
                break;
            }
        }

        if (failed || mGenerator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
            // interval detection failed
            return Double.MAX_VALUE;
        }

        // check that enough measurements have been obtained
        if (mAccelerometerMeasurements.size() < mAccelerometerCalibrator.getMinimumRequiredMeasurements()
                || mGyroscopeSequences.size() < mGyroscopeCalibrator.getMinimumRequiredMeasurementsOrSequences()
                || mMagnetometerMeasurements.size() < mMagnetometerCalibrator.getMinimumRequiredMeasurements()) {
            return Double.MAX_VALUE;
        }

        // set calibrator measurements
        switch (mAccelerometerCalibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_KINEMATICS:
                if (mAccelerometerCalibrator.isOrderedMeasurementsRequired()) {
                    final OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator calibrator =
                            (OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator) mAccelerometerCalibrator;

                    calibrator.setMeasurements(mAccelerometerMeasurements);

                } else {
                    final UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator calibrator =
                            (UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator) mAccelerometerCalibrator;

                    calibrator.setMeasurements(mAccelerometerMeasurements);
                }

                if (mAccelerometerCalibrator.isQualityScoresRequired()) {
                    final QualityScoredAccelerometerCalibrator calibrator =
                            (QualityScoredAccelerometerCalibrator) mAccelerometerCalibrator;

                    final int size = mAccelerometerMeasurements.size();
                    final double[] qualityScores = new double[size];
                    for (int i = 0; i < size; i++) {
                        qualityScores[i] = mAccelerometerQualityScoreMapper.map(
                                mAccelerometerMeasurements.get(i));
                    }
                    calibrator.setQualityScores(qualityScores);
                }
                break;
            case FRAME_BODY_KINEMATICS:
                // throw exception. Cannot use frames
            case STANDARD_DEVIATION_FRAME_BODY_KINEMATICS:
                // throw exception. Cannot use frames
            default:
                throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        if (mGyroscopeCalibrator.getMeasurementOrSequenceType() ==
                GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            final OrderedBodyKinematicsSequenceGyroscopeCalibrator calibrator =
                    (OrderedBodyKinematicsSequenceGyroscopeCalibrator) mGyroscopeCalibrator;

            calibrator.setSequences(mGyroscopeSequences);
        } else {
            throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        if (mGyroscopeCalibrator.isQualityScoresRequired()) {
            final QualityScoredGyroscopeCalibrator calibrator =
                    (QualityScoredGyroscopeCalibrator) mGyroscopeCalibrator;

            final int size = mGyroscopeSequences.size();
            final double[] qualityScores = new double[size];
            for (int i = 0; i < size; i++) {
                qualityScores[i] = mGyroscopeQualityScoreMapper.map(
                        mGyroscopeSequences.get(i));
            }
            calibrator.setQualityScores(qualityScores);
        }

        switch (mMagnetometerCalibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY:
                if (mMagnetometerCalibrator.isOrderedMeasurementsRequired()) {
                    final OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator calibrator =
                            (OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator)
                                    mMagnetometerCalibrator;

                    calibrator.setMeasurements(mMagnetometerMeasurements);
                } else {
                    final UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator calibrator =
                            (UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator)
                                    mMagnetometerCalibrator;

                    calibrator.setMeasurements(mMagnetometerMeasurements);
                }

                if (mMagnetometerCalibrator.isQualityScoresRequired()) {
                    final QualityScoredMagnetometerCalibrator calibrator =
                            (QualityScoredMagnetometerCalibrator) mMagnetometerCalibrator;

                    final int size = mMagnetometerMeasurements.size();
                    final double[] qualityScores = new double[size];
                    for (int i = 0; i < size; i++) {
                        qualityScores[i] = mMagnetometerQualityScoreMapper.map(
                                mMagnetometerMeasurements.get(i));
                    }
                    calibrator.setQualityScores(qualityScores);
                }
                break;
            case FRAME_BODY_MAGNETIC_FLUX_DENSITY:
                // throw exception. Cannot use frames
            case STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY:
                // throw exception. Cannot use frames
            default:
                throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        mAccelerometerCalibrator.calibrate();
        mGyroscopeCalibrator.calibrate();
        mMagnetometerCalibrator.calibrate();

        final double accelerometerMse = mAccelerometerCalibrator.getEstimatedMse();
        final double gyroscopeMse = mGyroscopeCalibrator.getEstimatedMse();
        final double magnetometerMse = mMagnetometerCalibrator.getEstimatedMse();

        // convert accelerometer and gyroscope mse to global mse
        final double mse = mMseRule.evaluate(accelerometerMse, gyroscopeMse, magnetometerMse);
        if (mse < mMinMse) {
            keepBestResult(mse, thresholdFactor);
        }
        return mse;
    }

    /**
     * Initializes accelerometer, gyroscope and magnetometer measurement generator to
     * convert {@link TimedBodyKinematicsAndMagneticFluxDensity} measurements after
     * interval detection into measurements and sequences used for accelerometer,
     * gyroscope and magnetometer calibration.
     */
    private void initialize() {
        final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener generatorListener =
                new AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener() {
                    @Override
                    public void onInitializationStarted(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onInitializationCompleted(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                            final double accelerometerBaseNoiseLevel) {
                        // not needed
                    }

                    @Override
                    public void onError(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                            final TriadStaticIntervalDetector.ErrorReason reason) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalDetected(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalDetected(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalSkipped(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalSkipped(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onGeneratedAccelerometerMeasurement(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                            final StandardDeviationBodyKinematics measurement) {
                        mAccelerometerMeasurements.add(measurement);
                    }

                    @Override
                    public void onGeneratedGyroscopeMeasurement(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                        mGyroscopeSequences.add(sequence);
                    }

                    @Override
                    public void onGeneratedMagnetometerMeasurement(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                            final StandardDeviationBodyMagneticFluxDensity measurement) {
                        mMagnetometerMeasurements.add(measurement);
                    }

                    @Override
                    public void onReset(
                            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }
                };

        mGenerator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(generatorListener);
    }

    /**
     * Keeps best calibration solution found so far.
     *
     * @param mse             Estimated Mean Square Error during calibration.
     * @param thresholdFactor threshold factor to be kept.
     */
    private void keepBestResult(final double mse, final double thresholdFactor) {
        mMinMse = mse;
        mOptimalThresholdFactor = thresholdFactor;

        mAngularSpeedNoiseRootPsd = mGenerator.getGyroscopeBaseNoiseLevelRootPsd();
        mBaseNoiseLevel = mGenerator.getAccelerometerBaseNoiseLevel();
        mThreshold = mGenerator.getThreshold();

        if (mEstimatedAccelerometerCovariance == null) {
            mEstimatedAccelerometerCovariance = new Matrix(mAccelerometerCalibrator.getEstimatedCovariance());
        } else {
            mEstimatedAccelerometerCovariance.copyFrom(mAccelerometerCalibrator.getEstimatedCovariance());
        }
        if (mEstimatedAccelerometerMa == null) {
            mEstimatedAccelerometerMa = new Matrix(mAccelerometerCalibrator.getEstimatedMa());
        } else {
            mEstimatedAccelerometerMa.copyFrom(mAccelerometerCalibrator.getEstimatedMa());
        }
        if (mAccelerometerCalibrator instanceof UnknownBiasAccelerometerCalibrator) {
            mEstimatedAccelerometerBiases = ((UnknownBiasAccelerometerCalibrator) mAccelerometerCalibrator)
                    .getEstimatedBiases();
        } else if (mAccelerometerCalibrator instanceof KnownBiasAccelerometerCalibrator) {
            mEstimatedAccelerometerBiases = ((KnownBiasAccelerometerCalibrator) mAccelerometerCalibrator)
                    .getBias();
        }

        if (mEstimatedGyroscopeCovariance == null) {
            mEstimatedGyroscopeCovariance = new Matrix(mGyroscopeCalibrator.getEstimatedCovariance());
        } else {
            mEstimatedGyroscopeCovariance.copyFrom(mGyroscopeCalibrator.getEstimatedCovariance());
        }
        if (mEstimatedGyroscopeMg == null) {
            mEstimatedGyroscopeMg = new Matrix(mGyroscopeCalibrator.getEstimatedMg());
        } else {
            mEstimatedGyroscopeMg.copyFrom(mGyroscopeCalibrator.getEstimatedMg());
        }
        if (mEstimatedGyroscopeGg == null) {
            mEstimatedGyroscopeGg = new Matrix(mGyroscopeCalibrator.getEstimatedGg());
        } else {
            mEstimatedGyroscopeGg.copyFrom(mGyroscopeCalibrator.getEstimatedGg());
        }
        if (mGyroscopeCalibrator instanceof UnknownBiasGyroscopeCalibrator) {
            mEstimatedGyroscopeBiases = ((UnknownBiasGyroscopeCalibrator) mGyroscopeCalibrator)
                    .getEstimatedBiases();
        } else if (mGyroscopeCalibrator instanceof KnownBiasAccelerometerCalibrator) {
            mEstimatedGyroscopeBiases = ((KnownBiasGyroscopeCalibrator) mGyroscopeCalibrator)
                    .getBias();
        }

        if (mEstimatedMagnetometerCovariance == null) {
            mEstimatedMagnetometerCovariance = new Matrix(mMagnetometerCalibrator.getEstimatedCovariance());
        } else {
            mEstimatedMagnetometerCovariance.copyFrom(mMagnetometerCalibrator.getEstimatedCovariance());
        }
        if (mEstimatedMagnetometerMm == null) {
            mEstimatedMagnetometerMm = new Matrix(mMagnetometerCalibrator.getEstimatedMm());
        } else {
            mEstimatedMagnetometerMm.copyFrom(mMagnetometerCalibrator.getEstimatedMm());
        }
        if (mMagnetometerCalibrator instanceof UnknownHardIronMagnetometerCalibrator) {
            mEstimatedMagnetometerHardIron = ((UnknownHardIronMagnetometerCalibrator) mMagnetometerCalibrator)
                    .getEstimatedHardIron();
        } else if (mMagnetometerCalibrator instanceof KnownHardIronMagnetometerCalibrator) {
            mEstimatedMagnetometerHardIron = ((KnownHardIronMagnetometerCalibrator) mMagnetometerCalibrator)
                    .getHardIron();
        }
    }

    /**
     * Creates an acceleration instance using provided value and unit.
     *
     * @param value value of measurement.
     * @param unit  unit of value.
     * @return created acceleration.
     */
    private Acceleration createMeasurement(
            final double value, final AccelerationUnit unit) {
        return new Acceleration(value, unit);
    }

    /**
     * Gets default unit for acceleration, which is meters per
     * squared second (m/s^2).
     *
     * @return default unit for acceleration.
     */
    private AccelerationUnit getDefaultUnit() {
        return AccelerationUnit.METERS_PER_SQUARED_SECOND;
    }
}
