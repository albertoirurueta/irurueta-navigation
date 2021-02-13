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
import com.irurueta.navigation.inertial.calibration.GyroscopeBiasUncertaintySource;
import com.irurueta.navigation.inertial.calibration.GyroscopeCalibrationSource;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.GyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.GyroscopeMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.OrderedBodyKinematicsSequenceGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QualityScoredGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.UnknownBiasGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;

import java.util.ArrayList;
import java.util.List;

/**
 * Optimizes threshold factor for interval detection of gyroscope data based
 * on results of calibration.
 * Implementations of this class will attempt to find best threshold factor
 * between provided range of values.
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 */
public abstract class GyroscopeIntervalDetectorThresholdFactorOptimizer extends
        IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematics,
                GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource>
        implements GyroscopeNoiseRootPsdSource, GyroscopeBiasUncertaintySource,
        GyroscopeCalibrationSource, AccelerometerNoiseRootPsdSource {

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
     * Gyroscope calibrator.
     */
    private GyroscopeNonLinearCalibrator mCalibrator;

    /**
     * A measurement generator for gyroscope calibrators.
     */
    private GyroscopeMeasurementsGenerator mGenerator;

    /**
     * Generated sequences to be used for gyroscope calibration.
     */
    private List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mSequences;

    /**
     * Mapper to convert {@link BodyKinematicsSequence} sequences of {@link StandardDeviationTimedBodyKinematics}
     * into quality scores.
     */
    private QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mQualityScoreMapper =
            new DefaultGyroscopeQualityScoreMapper();

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
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
     */
    private double[] mEstimatedBiases;

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
    private Matrix mEstimatedMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix mEstimatedGg;

    /**
     * Constructor.
     */
    public GyroscopeIntervalDetectorThresholdFactorOptimizer() {
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public GyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics}.
     */
    public GyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeNonLinearCalibrator calibrator) {
        try {
            setCalibrator(calibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics}.
     */
    public GyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final GyroscopeNonLinearCalibrator calibrator) {
        super(dataSource);
        try {
            setCalibrator(calibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Gets provided gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return gyroscope calibrator to be used to optimize its MSE.
     */
    public GyroscopeNonLinearCalibrator getCalibrator() {
        return mCalibrator;
    }

    /**
     * Sets gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @param calibrator gyroscope calibrator to be use dto optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics}.
     */
    public void setCalibrator(final GyroscopeNonLinearCalibrator calibrator)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (calibrator != null && calibrator.getMeasurementOrSequenceType() !=
                GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            throw new IllegalArgumentException();
        }

        mCalibrator = calibrator;
    }

    /**
     * Gets mapper to convert {@link BodyKinematicsSequence} sequences of {@link StandardDeviationTimedBodyKinematics}
     * into quality scores.
     *
     * @return mapper to convert sequences into quality scores.
     */
    public QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> getQualityScoreMapper() {
        return mQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link BodyKinematicsSequence} sequences of {@link StandardDeviationTimedBodyKinematics}
     * into quality scores.
     *
     * @param qualityScoreMapper mapper to convert seqiemces into quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setQualityScoreMapper(
            final QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> qualityScoreMapper)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mQualityScoreMapper = qualityScoreMapper;
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
        return super.isReady() && mCalibrator != null && mQualityScoreMapper != null;
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
     * Gets norm of estimated standard deviation of gyroscope bias expressed in
     * radians per second (rad/s).
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    @Override
    public Double getEstimatedBiasStandardDeviationNorm() {
        return mEstimatedCovariance != null ?
                Math.sqrt(getEstimatedBiasXVariance() + getEstimatedBiasYVariance() + getEstimatedBiasZVariance()) :
                null;
    }

    /**
     * Gets variance of estimated x coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated x coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasXVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets variance of estimated y coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated y coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasYVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets variance of estimated z coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated z coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasZVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    @Override
    public double[] getEstimatedBiases() {
        return mEstimatedBiases;
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
    @Override
    public Matrix getEstimatedMg() {
        return mEstimatedMg;
    }

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    @Override
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
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
        if (mSequences == null) {
            mSequences = new ArrayList<>();
        } else {
            mSequences.clear();
        }

        mGenerator.reset();
        mGenerator.setThresholdFactor(thresholdFactor);

        int count = mDataSource.count();
        boolean failed = false;
        for (int i = 0; i < count; i++) {
            final TimedBodyKinematics bodyKinematics = mDataSource.getAt(i);
            if (!mGenerator.process(bodyKinematics)) {
                failed = true;
                break;
            }
        }

        if (failed || mGenerator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
            // interval detection failed
            return Double.MAX_VALUE;
        }

        // check that enough measurements have been obtained
        if (mSequences.size() < mCalibrator.getMinimumRequiredMeasurementsOrSequences()) {
            return Double.MAX_VALUE;
        }

        // set calibrator measurements
        if (mCalibrator.getMeasurementOrSequenceType() ==
                GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            final OrderedBodyKinematicsSequenceGyroscopeCalibrator calibrator =
                    (OrderedBodyKinematicsSequenceGyroscopeCalibrator) mCalibrator;

            calibrator.setSequences(mSequences);
        } else {
            throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        if (mCalibrator.isQualityScoresRequired()) {
            final QualityScoredGyroscopeCalibrator calibrator =
                    (QualityScoredGyroscopeCalibrator) mCalibrator;

            final int size = mSequences.size();
            final double[] qualityScores = new double[size];
            for (int i = 0; i < size; i++) {
                qualityScores[i] = mQualityScoreMapper.map(mSequences.get(i));
            }
            calibrator.setQualityScores(qualityScores);
        }

        mCalibrator.calibrate();

        final double mse = mCalibrator.getEstimatedMse();
        if (mse < mMinMse) {
            keepBestResult(mse, thresholdFactor);
        }
        return mse;
    }

    /**
     * Initializes gyroscope measurement generator to convert timed body kinematics
     * measurements after interval detection into measurements used for
     * gyroscope calibration.
     */
    private void initialize() {
        final GyroscopeMeasurementsGeneratorListener generatorListener =
                new GyroscopeMeasurementsGeneratorListener() {
                    @Override
                    public void onInitializationStarted(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onInitializationCompleted(
                            final GyroscopeMeasurementsGenerator generator,
                            final double baseNoiseLevel) {
                        // not needed
                    }

                    @Override
                    public void onError(
                            final GyroscopeMeasurementsGenerator generator,
                            final TriadStaticIntervalDetector.ErrorReason reason) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalDetected(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalDetected(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalSkipped(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalSkipped(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onGeneratedMeasurement(
                            final GyroscopeMeasurementsGenerator generator,
                            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                        mSequences.add(sequence);
                    }

                    @Override
                    public void onReset(
                            final GyroscopeMeasurementsGenerator generator) {
                        // not needed
                    }
                };

        mGenerator = new GyroscopeMeasurementsGenerator(generatorListener);
    }

    /**
     * Keeps best calibration solution found so far.
     *
     * @param mse Estimated Mean Square Error during calibration.
     * @param thresholdFactor threshold factor to be kept.
     */
    private void keepBestResult(final double mse, final double thresholdFactor) {
        mMinMse = mse;
        mOptimalThresholdFactor = thresholdFactor;

        mAngularSpeedNoiseRootPsd = mGenerator.getGyroscopeBaseNoiseLevelRootPsd();
        mBaseNoiseLevel = mGenerator.getAccelerometerBaseNoiseLevel();
        mThreshold = mGenerator.getThreshold();

        if (mEstimatedCovariance == null) {
            mEstimatedCovariance = new Matrix(mCalibrator.getEstimatedCovariance());
        } else {
            mEstimatedCovariance.copyFrom(mCalibrator.getEstimatedCovariance());
        }
        if (mEstimatedMg == null) {
            mEstimatedMg = new Matrix(mCalibrator.getEstimatedMg());
        } else {
            mEstimatedMg.copyFrom(mCalibrator.getEstimatedMg());
        }
        if (mEstimatedGg == null) {
            mEstimatedGg = new Matrix(mCalibrator.getEstimatedGg());
        } else {
            mEstimatedGg.copyFrom(mCalibrator.getEstimatedGg());
        }
        if (mCalibrator instanceof UnknownBiasGyroscopeCalibrator) {
            mEstimatedBiases = ((UnknownBiasGyroscopeCalibrator) mCalibrator)
                    .getEstimatedBiases();
        } else if (mCalibrator instanceof KnownBiasAccelerometerCalibrator) {
            mEstimatedBiases = ((KnownBiasGyroscopeCalibrator) mCalibrator)
                    .getBias();
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
