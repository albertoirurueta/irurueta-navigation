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
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGeneratorListener;
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
 * Optimizes threshold factor for interval detection of magnetometer data based
 * on results of calibration.
 * Implementations of this class will attempt to find best threshold factor
 * between provided range of values.
 * Only magnetometer calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 */
public abstract class MagnetometerIntervalDetectorThresholdFactorOptimizer extends
        IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> implements
        AccelerometerNoiseRootPsdSource {

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
     * Magnetometer calibrator.
     */
    private MagnetometerNonLinearCalibrator mCalibrator;

    /**
     * A measurement generator for magnetometer calibrators.
     */
    private MagnetometerMeasurementsGenerator mGenerator;

    /**
     * Generated measurements to be used for magnetometer calibration.
     */
    private List<StandardDeviationBodyMagneticFluxDensity> mMeasurements;

    /**
     * Mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements
     * into quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> mQualityScoreMapper =
            new DefaultMagnetometerQualityScoreMapper();

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
    private Matrix mEstimatedMm;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] mEstimatedHardIron;

    /**
     * Constructor.
     */
    public MagnetometerIntervalDetectorThresholdFactorOptimizer() {
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public MagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param calibrator a magnetometer calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public MagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerNonLinearCalibrator calibrator) {
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
     * @param calibrator a magnetometer calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public MagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final MagnetometerNonLinearCalibrator calibrator) {
        super(dataSource);
        try {
            setCalibrator(calibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Gets provided magnetometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return magnetometer calibrator to be used to optimize its MSE.
     */
    public MagnetometerNonLinearCalibrator getCalibrator() {
        return mCalibrator;
    }

    /**
     * Sets magnetometer calibrator to be used to optimize its Mean Square Error.
     *
     * @param calibrator magnetometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public void setCalibrator(final MagnetometerNonLinearCalibrator calibrator)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (calibrator != null && calibrator.getMeasurementType() !=
                MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY) {
            throw new IllegalArgumentException();
        }

        mCalibrator = calibrator;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements into
     * quality scores.
     *
     * @return mapper to convert measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> getQualityScoreMapper() {
        return mQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements into
     * quality scores.
     *
     * @param qualityScoreMapper mapper to convert measurements into quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper)
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
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3) of best solution that has been found.
     *
     * @return accelerometer base noise level PSD of best solution that has
     * been found.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return mBaseNoiseLevel * mBaseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5) of best solution that has been found.
     *
     * @return accelerometer base noise level root PSD of best solution that has
     * been found.
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
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    public double[] getEstimatedHardIron() {
        return mEstimatedHardIron;
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
    public Matrix getEstimatedMm() {
        return mEstimatedMm;
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
        if (mMeasurements == null) {
            mMeasurements = new ArrayList<>();
        } else {
            mMeasurements.clear();
        }

        mGenerator.reset();
        mGenerator.setThresholdFactor(thresholdFactor);

        int count = mDataSource.count();
        boolean failed = false;
        for (int i = 0; i < count; i++) {
            final BodyKinematicsAndMagneticFluxDensity bodyKinematics =
                    mDataSource.getAt(i);
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
        if (mMeasurements.size() < mCalibrator.getMinimumRequiredMeasurements()) {
            return Double.MAX_VALUE;
        }

        // set calibrator measurements
        switch (mCalibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY:
                if (mCalibrator.isOrderedMeasurementsRequired()) {
                    final OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator calibrator =
                            (OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator) mCalibrator;

                    calibrator.setMeasurements(mMeasurements);
                } else {
                    final UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator calibrator =
                            (UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator) mCalibrator;

                    calibrator.setMeasurements(mMeasurements);
                }

                if (mCalibrator.isQualityScoresRequired()) {
                    final QualityScoredMagnetometerCalibrator calibrator =
                            (QualityScoredMagnetometerCalibrator) mCalibrator;

                    final int size = mMeasurements.size();
                    final double[] qualityScores = new double[size];
                    for (int i = 0; i < size; i++) {
                        qualityScores[i] = mQualityScoreMapper.map(mMeasurements.get(i));
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

        mCalibrator.calibrate();

        final double mse = mCalibrator.getEstimatedMse();
        if (mse < mMinMse) {
            keepBestResult(mse, thresholdFactor);
        }
        return mse;
    }

    /**
     * Initializes magnetometer measurement generator to convert
     * body kinematics + magnetic flux measurements after interval detection into
     * measurements used for magnetometer calibration.
     */
    private void initialize() {
        final MagnetometerMeasurementsGeneratorListener generatorListener =
                new MagnetometerMeasurementsGeneratorListener() {
                    @Override
                    public void onInitializationStarted(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onInitializationCompleted(
                            final MagnetometerMeasurementsGenerator generator,
                            final double baseNoiseLevel) {
                        // not needed
                    }

                    @Override
                    public void onError(
                            final MagnetometerMeasurementsGenerator generator,
                            final TriadStaticIntervalDetector.ErrorReason reason) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalDetected(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalDetected(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onStaticIntervalSkipped(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onDynamicIntervalSkipped(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }

                    @Override
                    public void onGeneratedMeasurement(
                            final MagnetometerMeasurementsGenerator generator,
                            final StandardDeviationBodyMagneticFluxDensity measurement) {
                        mMeasurements.add(measurement);
                    }

                    @Override
                    public void onReset(
                            final MagnetometerMeasurementsGenerator generator) {
                        // not needed
                    }
                };

        mGenerator = new MagnetometerMeasurementsGenerator(generatorListener);
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

        mBaseNoiseLevel = mGenerator.getAccelerometerBaseNoiseLevel();
        mThreshold = mGenerator.getThreshold();

        if (mEstimatedCovariance == null) {
            mEstimatedCovariance = new Matrix(mCalibrator.getEstimatedCovariance());
        } else {
            mEstimatedCovariance.copyFrom(mCalibrator.getEstimatedCovariance());
        }
        if (mEstimatedMm == null) {
            mEstimatedMm = new Matrix(mCalibrator.getEstimatedMm());
        } else {
            mEstimatedMm.copyFrom(mCalibrator.getEstimatedMm());
        }
        if (mCalibrator instanceof UnknownHardIronMagnetometerCalibrator) {
            mEstimatedHardIron = ((UnknownHardIronMagnetometerCalibrator) mCalibrator)
                    .getEstimatedHardIron();
        } else if (mCalibrator instanceof KnownHardIronMagnetometerCalibrator) {
            mEstimatedHardIron = ((KnownHardIronMagnetometerCalibrator) mCalibrator)
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
