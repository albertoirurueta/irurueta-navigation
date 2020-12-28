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

package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetectorListener;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Base class to generate measurements for the calibration of accelerometers, gyroscopes or
 * magnetometers after detection of static/dynamic intervals.
 *
 * @param <T> type of measurement to be generated.
 * @param <G> type of generator.
 * @param <L> type of listener.
 * @param <I> type of input data to be processed.
 */
public abstract class MeasurementsGenerator<T, G extends MeasurementsGenerator<T, G, L, I>,
        L extends MeasurementsGeneratorListener<T, G, L, I>, I> {

    /**
     * Default minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     */
    public static final int DEFAULT_MIN_STATIC_SAMPLES = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

    /**
     * Default maximum number of samples allowed in dynamic intervals.
     * Larger dynamic intervals will be discarded.
     */
    public static final int DEFAULT_MAX_DYNAMIC_SAMPLES = 30 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

    /**
     * Listener to handle generated events.
     */
    protected L mListener;

    /**
     * An acceleration triad.
     * This is reused for memory efficiency.
     */
    protected final AccelerationTriad mTriad = new AccelerationTriad();

    /**
     * Static/dynamic interval detector using accelerometer samples.
     */
    protected final AccelerationTriadStaticIntervalDetector mStaticIntervalDetector;

    /**
     * Indicates whether generator is running or not.
     */
    private boolean mRunning;

    /**
     * Minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     */
    private int mMinStaticSamples = DEFAULT_MIN_STATIC_SAMPLES;

    /**
     * Maximum number of samples allowed in dynamic intervals.
     * Larger dynamic intervals will be discarded.
     */
    private int mMaxDynamicSamples = DEFAULT_MAX_DYNAMIC_SAMPLES;

    /**
     * Number of samples that have been processed in a static period so far.
     */
    private int mProcessedStaticSamples;

    /**
     * Number of samples that have been processed in a dynamic period so far.
     */
    private int mProcessedDynamicSamples;

    /**
     * Indicates whether static interval must be skipped.
     */
    private boolean mSkipStaticInterval;

    /**
     * Indicates whether dynamic interval must be skipped.
     */
    private boolean mSkipDynamicInterval;

    /**
     * Constructor.
     */
    public MeasurementsGenerator() {
        mStaticIntervalDetector = new AccelerationTriadStaticIntervalDetector();
        try {
            setupListener();
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public MeasurementsGenerator(final L listener) {
        this();
        mListener = listener;
    }

    /**
     * Gets time interval between input samples expressed in seconds (s).
     *
     * @return time interval between input samples.
     */
    public double getTimeInterval() {
        return mStaticIntervalDetector.getTimeInterval();
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

        mStaticIntervalDetector.setTimeInterval(timeInterval);
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
        return mMinStaticSamples;
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

        if (minStaticSamples < WindowedTriadNoiseEstimator.MIN_WINDOW_SIZE) {
            throw new IllegalArgumentException();
        }

        mMinStaticSamples = minStaticSamples;
    }

    /**
     * Gets maximum number of samples allowed in dynamic intervals.
     *
     * @return maximum number of samples allowed in dynamic intervals.
     */
    public int getMaxDynamicSamples() {
        return mMaxDynamicSamples;
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

        if (maxDynamicSamples < WindowedTriadNoiseEstimator.MIN_WINDOW_SIZE) {
            throw new IllegalArgumentException();
        }

        mMaxDynamicSamples = maxDynamicSamples;
    }

    /**
     * Gets listener to handle generated events.
     *
     * @return listener to handle generated events.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle generated events.
     *
     * @param listener listener to handle generated evets.
     * @throws LockedException if generator is busy.
     */
    public void setListener(final L listener) throws LockedException {
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
        return mStaticIntervalDetector.getWindowSize();
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

        mStaticIntervalDetector.setWindowSize(windowSize);
    }

    /**
     * Gets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return mStaticIntervalDetector.getInitialStaticSamples();
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

        mStaticIntervalDetector.setInitialStaticSamples(initialStaticSamples);
    }

    /**
     * Gets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @return factor to be applied to detected base noise level.
     */
    public double getThresholdFactor() {
        return mStaticIntervalDetector.getThresholdFactor();
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

        mStaticIntervalDetector.setThresholdFactor(thresholdFactor);
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
        return mStaticIntervalDetector.getInstantaneousNoiseLevelFactor();
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

        mStaticIntervalDetector.setInstantaneousNoiseLevelFactor(
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
        return mStaticIntervalDetector.getBaseNoiseLevelAbsoluteThreshold();
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

        mStaticIntervalDetector.setBaseNoiseLevelAbsoluteThreshold(
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
        return mStaticIntervalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
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
        mStaticIntervalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
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

        mStaticIntervalDetector.setBaseNoiseLevelAbsoluteThreshold(
                baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets internal status of this generator.
     *
     * @return internal status of this generator.
     */
    public TriadStaticIntervalDetector.Status getStatus() {
        return mStaticIntervalDetector.getStatus();
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
        return mStaticIntervalDetector.getBaseNoiseLevel();
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
        return mStaticIntervalDetector.getBaseNoiseLevelAsMeasurement();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(
            final Acceleration result) {
        mStaticIntervalDetector.getBaseNoiseLevelAsMeasurement(result);
    }

    /**
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3).
     *
     * @return accelerometer base noise level PSD.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return mStaticIntervalDetector.getBaseNoiseLevelPsd();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return mStaticIntervalDetector.getBaseNoiseLevelRootPsd();
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2).
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public double getThreshold() {
        return mStaticIntervalDetector.getThreshold();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public Acceleration getThresholdAsMeasurement() {
        return mStaticIntervalDetector.getThresholdAsMeasurement();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final Acceleration result) {
        mStaticIntervalDetector.getThresholdAsMeasurement(result);
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     *
     * @return number of samples that have been processed in a static period so far.
     */
    public int getProcessedStaticSamples() {
        return mProcessedStaticSamples;
    }

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     *
     * @return number of samples that have been processed in a dynamic period so far.
     */
    public int getProcessedDynamicSamples() {
        return mProcessedDynamicSamples;
    }

    /**
     * Indicates whether last static interval must be skipped.
     *
     * @return true if last static interval must be skipped.
     */
    public boolean isStaticIntervalSkipped() {
        return mSkipStaticInterval;
    }

    /**
     * Indicates whether last dynamic interval must be skipped.
     *
     * @return true if last dynamic interval must be skipped.
     */
    public boolean isDynamicIntervalSkipped() {
        return mSkipDynamicInterval;
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
    public boolean process(final I sample) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;
        checkProcessedSamples();

        getAccelerationTriadFromInputSample(sample);
        final boolean result = mStaticIntervalDetector.process(mTriad);

        if (result) {
            updateCounters();
            postProcess(sample);
        }
        mRunning = false;

        return result;
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

        mStaticIntervalDetector.reset();

        mProcessedDynamicSamples = 0;
        mProcessedStaticSamples = 0;

        mSkipDynamicInterval = false;
        mSkipStaticInterval = false;

        if (mListener != null) {
            //noinspection unchecked
            mListener.onReset((G) this);
        }
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    protected abstract void postProcess(final I sample) throws LockedException;

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #mTriad}.
     *
     * @param sample input sample.
     */
    protected abstract void getAccelerationTriadFromInputSample(
            final I sample);

    /**
     * Handles a static-to-dynamic interval change.
     *
     * @param accumulatedAvgX average x-coordinate of measurements during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedAvgY average y-coordinate of specific force during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedAvgZ average z-coordinate of specific force during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedStdX standard deviation of x-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     * @param accumulatedStdY standard deviation of y-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     * @param accumulatedStdZ standard deviation of z-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     */
    protected abstract void handleStaticToDynamicChange(
            final double accumulatedAvgX,
            final double accumulatedAvgY,
            final double accumulatedAvgZ,
            final double accumulatedStdX,
            final double accumulatedStdY,
            final double accumulatedStdZ);

    /**
     * Handles a dynamic-to-static interval change.
     */
    protected abstract void handleDynamicToStaticChange();

    /**
     * Handles an initialization completion.
     */
    protected abstract void handleInitializationCompleted();

    /**
     * Handles an error during initialization.
     */
    protected abstract void handleInitializationFailed();

    /**
     * Check processed samples so far before processing a new one.
     */
    protected void checkProcessedSamples() {
        if (mProcessedDynamicSamples > mMaxDynamicSamples) {
            final boolean wasSkipped = mSkipDynamicInterval;
            mSkipDynamicInterval = true;

            if (mListener != null && !wasSkipped) {
                //noinspection unchecked
                mListener.onDynamicIntervalSkipped((G) this);
            }
        }
    }

    /**
     * Updates counters of processed samples.
     */
    protected void updateCounters() {
        switch (mStaticIntervalDetector.getStatus()) {
            case STATIC_INTERVAL:
                mProcessedStaticSamples++;
                mProcessedDynamicSamples = 0;
                break;
            case DYNAMIC_INTERVAL:
                mProcessedDynamicSamples++;
                mProcessedStaticSamples = 0;
                break;
        }
    }

    /**
     * Setups listener for static interval detector.
     *
     * @throws LockedException if static interval detector is busy.
     */
    private void setupListener() throws LockedException {
        final AccelerationTriadStaticIntervalDetectorListener listener =
                new AccelerationTriadStaticIntervalDetectorListener() {
                    @Override
                    public void onInitializationStarted(
                            final AccelerationTriadStaticIntervalDetector detector) {

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onInitializationStarted((G) MeasurementsGenerator.this);
                        }
                    }

                    @Override
                    public void onInitializationCompleted(
                            final AccelerationTriadStaticIntervalDetector detector,
                            final double baseNoiseLevel) {

                        handleInitializationCompleted();

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onInitializationCompleted(
                                    (G) MeasurementsGenerator.this,
                                    baseNoiseLevel);
                        }
                    }

                    @Override
                    public void onError(
                            final AccelerationTriadStaticIntervalDetector detector,
                            final double accumulatedNoiseLevel,
                            final double instantaneousNoiseLevel,
                            final TriadStaticIntervalDetector.ErrorReason reason) {

                        handleInitializationFailed();

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onError((G) MeasurementsGenerator.this, reason);
                        }
                    }

                    @Override
                    public void onStaticIntervalDetected(
                            final AccelerationTriadStaticIntervalDetector detector,
                            final double instantaneousAvgX,
                            final double instantaneousAvgY,
                            final double instantaneousAvgZ,
                            final double instantaneousStdX,
                            final double instantaneousStdY,
                            final double instantaneousStdZ) {

                        handleDynamicToStaticChange();
                        mSkipDynamicInterval = false;

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onStaticIntervalDetected((G) MeasurementsGenerator.this);
                        }
                    }

                    @Override
                    public void onDynamicIntervalDetected(
                            final AccelerationTriadStaticIntervalDetector detector,
                            final double instantaneousAvgX,
                            final double instantaneousAvgY,
                            final double instantaneousAvgZ,
                            final double instantaneousStdX,
                            final double instantaneousStdY,
                            final double instantaneousStdZ,
                            final double accumulatedAvgX,
                            final double accumulatedAvgY,
                            final double accumulatedAvgZ,
                            final double accumulatedStdX,
                            final double accumulatedStdY,
                            final double accumulatedStdZ) {

                        if (mProcessedStaticSamples < mMinStaticSamples) {
                            final boolean wasSkipped = mSkipStaticInterval;
                            mSkipStaticInterval = true;

                            if (mListener != null && !wasSkipped) {
                                //noinspection unchecked
                                mListener.onStaticIntervalSkipped((G) MeasurementsGenerator.this);
                            }
                        }

                        handleStaticToDynamicChange(
                                accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ,
                                accumulatedStdX, accumulatedStdY, accumulatedStdZ);
                        mSkipStaticInterval = false;

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onDynamicIntervalDetected((G) MeasurementsGenerator.this);
                        }
                    }

                    @Override
                    public void onReset(
                            final AccelerationTriadStaticIntervalDetector detector) {
                    }
                };
        mStaticIntervalDetector.setListener(listener);
    }
}
