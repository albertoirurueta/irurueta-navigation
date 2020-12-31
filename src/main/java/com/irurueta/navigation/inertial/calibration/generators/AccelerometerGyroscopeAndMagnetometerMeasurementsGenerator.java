package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Generates measurements for the calibration of accelerometers and gyroscopes by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with accelerometer calibrators based
 * on the knowledge of gravity norm (or Earth position) when the device orientation
 * is unknown, with easy gyroscope calibrators and with magnetometer calibrators based
 * on the knowledge of position on Earth and time instant.
 * Notice that accuracy of the gyroscope calibration is very sensitive to the
 * accuracy of detected dynamic intervals respect the average specific forces
 * during static intervals.
 * In order to increase the accuracy, calibration should be repeated trying different
 * threshold factors {@link #getThresholdFactor()}.
 *
 * @see AccelerometerMeasurementsGenerator
 * @see GyroscopeMeasurementsGenerator
 * @see MagnetometerMeasurementsGenerator
 */
public class AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator implements
        AccelerometerNoiseRootPsdSource, GyroscopeNoiseRootPsdSource {

    /**
     * Listener to handle generated events.
     */
    private AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener mListener;

    /**
     * Listener for internal accelerometer measurements generator.
     */
    private final AccelerometerMeasurementsGeneratorListener mAccelerometerListener =
            new AccelerometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerMeasurementsGenerator generator,
                        final double baseNoiseLevel) {
                    // no action required
                }

                @Override
                public void onError(
                        final AccelerometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    if (mListener != null) {
                        mListener.onError(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                                reason);
                    }
                }

                @Override
                public void onStaticIntervalDetected(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalDetected(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onStaticIntervalSkipped(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalSkipped(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onGeneratedMeasurement(
                        final AccelerometerMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    if (mListener != null) {
                        mListener.onGeneratedAccelerometerMeasurement(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                                measurement);
                    }
                }

                @Override
                public void onReset(
                        final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }
            };

    /**
     * Listener for internal gyroscope measurements generator.
     */
    private final GyroscopeMeasurementGeneratorListener mGyroscopeListener =
            new GyroscopeMeasurementGeneratorListener() {
                @Override
                public void onInitializationStarted(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onInitializationCompleted(
                        final GyroscopeMeasurementsGenerator generator,
                        final double baseNoiseLevel) {
                    // no action required
                }

                @Override
                public void onError(
                        final GyroscopeMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // no action required
                }

                @Override
                public void onStaticIntervalDetected(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalDetected(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onStaticIntervalSkipped(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalSkipped(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onGeneratedMeasurement(
                        final GyroscopeMeasurementsGenerator generator,
                        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
                    if (mListener != null) {
                        mListener.onGeneratedGyroscopeMeasurement(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                                measurement);
                    }
                }

                @Override
                public void onReset(
                        final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }
            };

    /**
     * Listener for internal magnetometer measurements generator.
     */
    private final MagnetometerMeasurementsGeneratorListener mMagnetometerListener =
            new MagnetometerMeasurementsGeneratorListener() {
        @Override
        public void onInitializationStarted(
                final MagnetometerMeasurementsGenerator generator) {
            if (mListener != null) {
                mListener.onInitializationStarted(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
            }
        }

        @Override
        public void onInitializationCompleted(
                final MagnetometerMeasurementsGenerator generator,
                final double baseNoiseLevel) {
            if (mListener != null) {
                mListener.onInitializationCompleted(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                        baseNoiseLevel);
            }
        }

        @Override
        public void onError(
                final MagnetometerMeasurementsGenerator generator,
                final TriadStaticIntervalDetector.ErrorReason reason) {
            // no action required
        }

        @Override
        public void onStaticIntervalDetected(
                final MagnetometerMeasurementsGenerator generator) {
            if (mListener != null) {
                mListener.onStaticIntervalDetected(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
            }
        }

        @Override
        public void onDynamicIntervalDetected(
                final MagnetometerMeasurementsGenerator generator) {
            if (mListener != null) {
                mListener.onDynamicIntervalDetected(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
            }
        }

        @Override
        public void onStaticIntervalSkipped(
                final MagnetometerMeasurementsGenerator generator) {
            if (mListener != null) {
                mListener.onStaticIntervalSkipped(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
            }
        }

        @Override
        public void onDynamicIntervalSkipped(
                final MagnetometerMeasurementsGenerator generator) {
            if (mListener != null) {
                mListener.onDynamicIntervalSkipped(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
            }
        }

        @Override
        public void onGeneratedMeasurement(
                final MagnetometerMeasurementsGenerator generator,
                final StandardDeviationBodyMagneticFluxDensity measurement) {
            if (mListener != null) {
                mListener.onGeneratedMagnetometerMeasurement(
                        AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                        measurement);
            }
        }

        @Override
        public void onReset(
                final MagnetometerMeasurementsGenerator generator) {
            // no action required
        }
    };

    /**
     * Internal accelerometer measurements generator.
     */
    private final AccelerometerMeasurementsGenerator mAccelerometerMeasurementsGenerator =
            new AccelerometerMeasurementsGenerator(mAccelerometerListener);

    /**
     * Internal gyroscope measurements generator.
     */
    private final GyroscopeMeasurementsGenerator mGyroscopeMeasurementsGenerator =
            new GyroscopeMeasurementsGenerator(mGyroscopeListener);

    /**
     * Internal magnetometer measurements generator.
     */
    private final MagnetometerMeasurementsGenerator mMagnetometerMeasurementsGenerator =
            new MagnetometerMeasurementsGenerator(mMagnetometerListener);

    /**
     * Indicates whether generator is running or not.
     */
    private boolean mRunning;

    /**
     * Timed body kinematics instance to be reused.
     */
    private final TimedBodyKinematics mTimedKinematics = new TimedBodyKinematics();

    /**
     * Constructor.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Gets time interval between input samples expressed in seconds (s).
     *
     * @return time interval between input samples.
     */
    public double getTimeInterval() {
        return mAccelerometerMeasurementsGenerator.getTimeInterval();
    }

    /**
     * Sets time interval between input samples expressed in seconds (s).
     *
     * @param timeInterval time interval between input samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if generator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        mAccelerometerMeasurementsGenerator.setTimeInterval(timeInterval);
        mGyroscopeMeasurementsGenerator.setTimeInterval(timeInterval);
        mMagnetometerMeasurementsGenerator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between input samples.
     *
     * @return time interval between input samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(getTimeInterval(), TimeUnit.SECOND);
    }

    /**
     * Gets time interval between input samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(getTimeInterval());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between input samples.
     *
     * @param timeInterval time interval between input samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     *
     * @return minimum number of samples required in a static interval to be taken into account.
     */
    public int getMinStaticSamples() {
        return mAccelerometerMeasurementsGenerator.getMinStaticSamples();
    }

    /**
     * Sets minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     *
     * @param minStaticSamples minimum number of samples required in a static interval to be
     *                         taken into account.
     * @throws LockedException          if generator is busy.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMinStaticSamples(final int minStaticSamples) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
        mGyroscopeMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
        mMagnetometerMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
    }

    /**
     * Gets maximum number of samples allowed in dynamic intervals.
     *
     * @return maximum number of samples allowed in dynamic intervals.
     */
    public int getMaxDynamicSamples() {
        return mAccelerometerMeasurementsGenerator.getMaxDynamicSamples();
    }

    /**
     * Sets maximum number of samples allowed in dynamic intervals.
     *
     * @param maxDynamicSamples maximum number of samples allowed in dynamic intervals.
     * @throws LockedException          if generator is busy.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMaxDynamicSamples(final int maxDynamicSamples) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
        mGyroscopeMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
        mMagnetometerMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
    }

    /**
     * Gets listener to handle generated events.
     *
     * @return listener to handle generated events.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle generated events.
     *
     * @param listener listener to handle generated evets.
     * @throws LockedException if generator is busy.
     */
    public void setListener(final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return mAccelerometerMeasurementsGenerator.getWindowSize();
    }

    /**
     * Sets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     * Window size must always be larger than allowed minimum value, which is 2 and
     * must have an odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if detector is busy processing a previous sample.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setWindowSize(windowSize);
        mGyroscopeMeasurementsGenerator.setWindowSize(windowSize);
        mMagnetometerMeasurementsGenerator.setWindowSize(windowSize);
    }

    /**
     * Gets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return mAccelerometerMeasurementsGenerator.getInitialStaticSamples();
    }

    /**
     * Sets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @param initialStaticSamples number of samples to be processed initially.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link TriadStaticIntervalDetector#MINIMUM_INITIAL_STATIC_SAMPLES}
     */
    public void setInitialStaticSamples(final int initialStaticSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
        mGyroscopeMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
        mMagnetometerMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
    }

    /**
     * Gets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @return factor to be applied to detected base noise level.
     */
    public double getThresholdFactor() {
        return mAccelerometerMeasurementsGenerator.getThresholdFactor();
    }

    /**
     * Sets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @param thresholdFactor factor to be applied to detected base noise level.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setThresholdFactor(final double thresholdFactor)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setThresholdFactor(thresholdFactor);
        mGyroscopeMeasurementsGenerator.setThresholdFactor(thresholdFactor);
        mMagnetometerMeasurementsGenerator.setThresholdFactor(thresholdFactor);
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
        return mAccelerometerMeasurementsGenerator.getInstantaneousNoiseLevelFactor();
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
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setInstantaneousNoiseLevelFactor(
            final double instantaneousNoiseLevelFactor) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setInstantaneousNoiseLevelFactor(
                instantaneousNoiseLevelFactor);
        mGyroscopeMeasurementsGenerator.setInstantaneousNoiseLevelFactor(
                instantaneousNoiseLevelFactor);
        mMagnetometerMeasurementsGenerator.setInstantaneousNoiseLevelFactor(
                instantaneousNoiseLevelFactor);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @return overall absolute threshold to determine whether there has
     * been excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return mAccelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThreshold();
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(
            final double baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
        mGyroscopeMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
        mMagnetometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @return overall absolute threshold to determine whether there has been
     * excessive motion.
     */
    public Acceleration getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return mAccelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            final Acceleration result) {
        mAccelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
                result);
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(
            final Acceleration baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
        mGyroscopeMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
        mMagnetometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets internal status of this generator.
     *
     * @return internal status of this generator.
     */
    public TriadStaticIntervalDetector.Status getStatus() {
        return mAccelerometerMeasurementsGenerator.getStatus();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization expressed in meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return accelerometer base noise level.
     */
    public double getAccelerometerBaseNoiseLevel() {
        return mAccelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevel();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return measurement base noise level.
     */
    public Acceleration getAccelerometerBaseNoiseLevelAsMeasurement() {
        return mAccelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelAsMeasurement();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(final Acceleration result) {
        mAccelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelAsMeasurement(result);
    }

    /**
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3).
     *
     * @return accelerometer base noise level PSD.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return mAccelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelPsd();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return mAccelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelRootPsd();
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2).
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public double getThreshold() {
        return mAccelerometerMeasurementsGenerator.getThreshold();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public Acceleration getThresholdAsMeasurement() {
        return mAccelerometerMeasurementsGenerator.getThresholdAsMeasurement();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final Acceleration result) {
        mAccelerometerMeasurementsGenerator.getThresholdAsMeasurement(result);
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     *
     * @return number of samples that have been processed in a static period so far.
     */
    public int getProcessedStaticSamples() {
        return mAccelerometerMeasurementsGenerator.getProcessedStaticSamples();
    }

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     *
     * @return number of samples that have been processed in a dynamic period so far.
     */
    public int getProcessedDynamicSamples() {
        return mAccelerometerMeasurementsGenerator.getProcessedDynamicSamples();
    }

    /**
     * Indicates whether last static interval must be skipped.
     *
     * @return true if last static interval must be skipped.
     */
    public boolean isStaticIntervalSkipped() {
        return mAccelerometerMeasurementsGenerator.isStaticIntervalSkipped();
    }

    /**
     * Indicates whether last dynamic interval must be skipped.
     *
     * @return true if last dynamic interval must be skipped.
     */
    public boolean isDynamicIntervalSkipped() {
        return mAccelerometerMeasurementsGenerator.isDynamicIntervalSkipped();
    }

    /**
     * Indicates whether generator is running or not.
     *
     * @return true if generator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Processes a sample of data.
     *
     * @param sample sample of data to be processed.
     * @return true if provided samples has been processed, false if provided triad has been skipped because
     * generator previously failed. If generator previously failed, it will need to be reset before
     * processing additional samples.
     * @throws LockedException if generator is busy processing a previous sample.
     */
    public boolean process(final TimedBodyKinematicsAndMagneticFluxDensity sample) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        try {
            mRunning = true;

            sample.getTimedKinematics(mTimedKinematics);

            return mAccelerometerMeasurementsGenerator.process(sample.getKinematics())
                    && mGyroscopeMeasurementsGenerator.process(mTimedKinematics)
                    && mMagnetometerMeasurementsGenerator.process(sample);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    public void reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerMeasurementsGenerator.reset();
        mGyroscopeMeasurementsGenerator.reset();
        mMagnetometerMeasurementsGenerator.reset();

        if (mListener != null) {
            mListener.onReset(this);
        }
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @return estimated average angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAvgAngularSpeedTriad() {
        return mGyroscopeMeasurementsGenerator.getInitialAvgAngularSpeedTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        mGyroscopeMeasurementsGenerator.getInitialAvgAngularSpeedTriad(result);
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @return estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAngularSpeedTriadStandardDeviation() {
        return mGyroscopeMeasurementsGenerator.getInitialAngularSpeedTriadStandardDeviation();
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviation(final AngularSpeedTriad result) {
        mGyroscopeMeasurementsGenerator.getInitialAngularSpeedTriadStandardDeviation(result);
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization expressed in radians per second (rad/s).
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @return gyroscope base noise level.
     */
    public double getGyroscopeBaseNoiseLevel() {
        return mGyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevel();
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @return gyroscope base noise level.
     */
    public AngularSpeed getGyroscopeBaseNoiseLevelAsMeasurement() {
        return mGyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelAsMeasurement();
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getGyroscopeBaseNoiseLevelAsMeasurement(
            final AngularSpeed result) {
        mGyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelAsMeasurement(
                result);
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return mGyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelPsd();
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return mGyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelRootPsd();
    }
}
