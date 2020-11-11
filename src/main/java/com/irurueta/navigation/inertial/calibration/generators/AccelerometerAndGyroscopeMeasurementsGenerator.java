package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;

public class AccelerometerAndGyroscopeMeasurementsGenerator {

    /**
     * Listener to handle generated events.
     */
    private AccelerometerAndGyroscopeMeasurementsGeneratorListener mListener;

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
                        AccelerometerAndGyroscopeMeasurementsGenerator.this,
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
                        AccelerometerAndGyroscopeMeasurementsGenerator.this,
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
                final GyroscopeMeasurementGenerator generator) {
            if (mListener != null) {
                mListener.onInitializationStarted(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this);
            }
        }

        @Override
        public void onInitializationCompleted(
                final GyroscopeMeasurementGenerator generator,
                final double baseNoiseLevel) {
            if (mListener != null) {
                mListener.onInitializationCompleted(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this,
                        baseNoiseLevel);
            }
        }

        @Override
        public void onError(
                final GyroscopeMeasurementGenerator generator,
                final TriadStaticIntervalDetector.ErrorReason reason) {
            // no action required
        }

        @Override
        public void onStaticIntervalDetected(
                final GyroscopeMeasurementGenerator generator) {
            if (mListener != null) {
                mListener.onStaticIntervalDetected(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this);
            }
        }

        @Override
        public void onDynamicIntervalDetected(
                final GyroscopeMeasurementGenerator generator) {
            if (mListener != null) {
                mListener.onDynamicIntervalDetected(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this);
            }
        }

        @Override
        public void onStaticIntervalSkipped(
                final GyroscopeMeasurementGenerator generator) {
            if (mListener != null) {
                mListener.onStaticIntervalSkipped(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this);
            }
        }

        @Override
        public void onDynamicIntervalSkipped(
                final GyroscopeMeasurementGenerator generator) {
            if (mListener != null) {
                mListener.onDynamicIntervalSkipped(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this);
            }
        }

        @Override
        public void onGeneratedMeasurement(
                final GyroscopeMeasurementGenerator generator,
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
            if (mListener != null) {
                mListener.onGeneratedGyroscopeMeasurement(
                        AccelerometerAndGyroscopeMeasurementsGenerator.this,
                        measurement);
            }
        }

        @Override
        public void onReset(
                final GyroscopeMeasurementGenerator generator) {
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
    private final GyroscopeMeasurementGenerator mGyroscopeMeasurementGenerator =
            new GyroscopeMeasurementGenerator(mGyroscopeListener);

    /**
     * Indicates whether generator is running or not.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public AccelerometerAndGyroscopeMeasurementsGenerator() {
    }

    /**
     * Constructor.
     * @param listener listener to handle events raised by this generator.
     */
    public AccelerometerAndGyroscopeMeasurementsGenerator(
            final AccelerometerAndGyroscopeMeasurementsGeneratorListener listener) {
        this();
        mListener = listener;
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
        mGyroscopeMeasurementGenerator.setMinStaticSamples(minStaticSamples);
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
        mGyroscopeMeasurementGenerator.setMaxDynamicSamples(maxDynamicSamples);
    }

    /**
     * Gets listener to handle generated events.
     *
     * @return listener to handle generated events.
     */
    public AccelerometerAndGyroscopeMeasurementsGeneratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle generated events.
     *
     * @param listener listener to handle generated evets.
     * @throws LockedException if generator is busy.
     */
    public void setListener(final AccelerometerAndGyroscopeMeasurementsGeneratorListener listener)
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
        mGyroscopeMeasurementGenerator.setWindowSize(windowSize);
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
        mGyroscopeMeasurementGenerator.setInitialStaticSamples(initialStaticSamples);
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
        mGyroscopeMeasurementGenerator.setThresholdFactor(thresholdFactor);
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
        mGyroscopeMeasurementGenerator.setInstantaneousNoiseLevelFactor(
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
        mGyroscopeMeasurementGenerator.setBaseNoiseLevelAbsoluteThreshold(
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
        mGyroscopeMeasurementGenerator.setBaseNoiseLevelAbsoluteThreshold(
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
        return mAccelerometerMeasurementsGenerator.getBaseNoiseLevel();
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
        return mAccelerometerMeasurementsGenerator.getBaseNoiseLevelAsMeasurement();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(final Acceleration result) {
        mAccelerometerMeasurementsGenerator.getBaseNoiseLevelAsMeasurement(result);
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
    public boolean process(final TimedBodyKinematics sample) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        try {
            mRunning = true;
            return mAccelerometerMeasurementsGenerator.process(sample.getKinematics())
                    && mGyroscopeMeasurementGenerator.process(sample);
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
        mGyroscopeMeasurementGenerator.reset();

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
        return mGyroscopeMeasurementGenerator.getInitialAvgAngularSpeedTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        mGyroscopeMeasurementGenerator.getInitialAvgAngularSpeedTriad(result);
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @return estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAngularSpeedTriadStandardDeviation() {
        return mGyroscopeMeasurementGenerator.getInitialAngularSpeedTriadStandardDeviation();
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviation(final AngularSpeedTriad result) {
        mGyroscopeMeasurementGenerator.getInitialAngularSpeedTriadStandardDeviation(result);
    }

    /**
     * Gets norm of estimated standard deviation of angular rate during initialization phase
     * expressed in radians per second (rad/s).
     *
     * @return norm of estimated standard deviation of angular rate during initialization phase.
     */
    public double getInitialAngularSpeedTriadStandardDeviationNorm() {
        return mGyroscopeMeasurementGenerator.getInitialAngularSpeedTriadStandardDeviationNorm();
    }

    /**
     * Gets norm of estimated standard deviation of angular rate during initialization phase.
     *
     * @return norm of estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeed getInitialAngularSpeedTriadStandardDeviationNormAsMeasurement() {
        return mGyroscopeMeasurementGenerator.getInitialAngularSpeedTriadStandardDeviationNormAsMeasurement();
    }

    /**
     * Gets norm of estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviationNormAsMeasurement(
            final AngularSpeed result) {
        mGyroscopeMeasurementGenerator.getInitialAngularSpeedTriadStandardDeviationNormAsMeasurement(
                result);
    }
}
