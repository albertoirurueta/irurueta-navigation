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
package com.irurueta.navigation.inertial.calibration.intervals;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.Triad;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.units.Measurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Abstract base class for detectors in charge of determining when a static period of
 * measurements starts and finishes.
 * Static periods are periods of time where the device is considered to
 * remain static (no movement applied to it).
 *
 * @param <U> type of unit.
 * @param <M> a type of measurement.
 * @param <T> a triad type.
 * @param <D> a detector type.
 * @param <L> a listener type.
 */
public abstract class TriadStaticIntervalDetector<U extends Enum<?>, M extends Measurement<U>,
        T extends Triad<U, M>, D extends TriadStaticIntervalDetector<U, M, T, D, L>,
        L extends TriadStaticIntervalDetectorListener<U, M, T, D>> {

    /**
     * Number of samples to keep within the window by default.
     * For a sensor generating 100 samples/second, this is equivalent to 1 second.
     * For a sensor generating 50 samples/second, this is equivalent to 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = WindowedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE;

    /**
     * Number of samples to process during the initial static period to determine the sensor
     * (accelerometer, gyroscope or magnetometer) noise level.
     * For a sensor generating 100 samples/second, this is equivalent to 50 seconds.
     * For a sensor generating 50 samples/second, this is equivalent to 100 seconds.
     */
    public static final int DEFAULT_INITIAL_STATIC_SAMPLES = 5000;

    /**
     * Minimum allowed number of samples to be processed during the initial static period.
     */
    public static final int MINIMUM_INITIAL_STATIC_SAMPLES = 2;

    /**
     * Default factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     */
    public static final double DEFAULT_THRESHOLD_FACTOR = 2.0;

    /**
     * Default factor to determine that a sudden movement has occurred during initialization
     * if instantaneous noise level exceeds accumulated noise level by this factor amount.
     * This factor is unit-less.
     */
    public static final double DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR = 2.0;

    /**
     * Default overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2) for acceleration, radians
     * per second (rad/s) for angular speed or Teslas (T) for magnetic flux density, and by
     * default it is set to the maximum allowed value, thus effectively disabling this error
     * condition check during initialization.
     */
    public static final double DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = Double.MAX_VALUE;

    /**
     * Number of samples to keep in window to find instantaneous noise level averaged within
     * the window of samples.
     * Window size should contain about 1 or 2 seconds of data to be averaged to obtain
     * a more reliable instantaneous noise level.
     */
    private int mWindowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     */
    private int mInitialStaticSamples = DEFAULT_INITIAL_STATIC_SAMPLES;

    /**
     * Factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     */
    private double mThresholdFactor = DEFAULT_THRESHOLD_FACTOR;

    /**
     * Factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount.
     * This factor is unit-less.
     */
    private double mInstantaneousNoiseLevelFactor = DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR;

    /**
     * Overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this threshold when
     * initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     */
    private double mBaseNoiseLevelAbsoluteThreshold = DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD;

    /**
     * Listener to handle events generated by this detector.
     */
    private L mListener;

    /**
     * Current status of this detector.
     */
    private Status mStatus = Status.IDLE;

    /**
     * Measurement base noise level that has been detected during initialization expressed in
     * meters per squared second (m/s^2) for acceleration, radians per second (rad/s) for
     * angular speed or Teslas (T) for magnetic flux density.
     */
    private double mBaseNoiseLevel;

    /**
     * Threshold to determine static/dynamic period changes expressed in meters per squared
     * second (m/s^2) for acceleration, radians per second (rad/s) for angular speed or
     * Teslas (T) for magnetic flux density.
     */
    private double mThreshold;

    /**
     * Indicates whether this detector is busy processing last provided sample.
     */
    private boolean mRunning;

    /**
     * Number of samples that have been processed so far.
     */
    private int mProcessedSamples;

    /**
     * Average x-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedAvgX;

    /**
     * Average y-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedAvgY;

    /**
     * Average z-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedAvgZ;

    /**
     * Standard deviation of x-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedStdX;

    /**
     * Standard deviation of y-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedStdY;

    /**
     * Standard deviation of z-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double mAccumulatedStdZ;

    /**
     * Windowed average x-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double mInstantaneousAvgX;

    /**
     * Windowed average y-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double mInstantaneousAvgY;

    /**
     * Windowed average z-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double mInstantaneousAvgZ;

    /**
     * Windowed standard deviation of x-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double mInstantaneousStdX;

    /**
     * Windowed standard deviation of y-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double mInstantaneousStdY;

    /**
     * Windowed standard deviation of z-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double mInstantaneousStdZ;

    /**
     * Estimator to find instantaneous measurement noise level averaged for a certain window of samples.
     */
    private final WindowedTriadNoiseEstimator<U, M, T, ?, ?> mWindowedNoiseEstimator;

    /**
     * Estimator to find accumulated accelerometer noise level.
     */
    private final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> mAccumulatedNoiseEstimator;

    /**
     * Constructor.
     *
     * @param windowedNoiseEstimator    windowed noise estimator to estimate noise within a window of measures.
     * @param accumulatedNoiseEstimator accumulated noise estimator to estimate accumulated noise and average.
     */
    protected TriadStaticIntervalDetector(
            final WindowedTriadNoiseEstimator<U, M, T, ?, ?> windowedNoiseEstimator,
            final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> accumulatedNoiseEstimator) {
        mWindowedNoiseEstimator = windowedNoiseEstimator;
        mAccumulatedNoiseEstimator = accumulatedNoiseEstimator;
    }

    /**
     * Constructor.
     *
     * @param windowedNoiseEstimator    windowed noise estimator to estimate noise within a window of measures.
     * @param accumulatedNoiseEstimator accumulated noise estimator to estimate accumulated noise and average.
     * @param listener                  listener to handle events generated by this detector.
     */
    protected TriadStaticIntervalDetector(
            final WindowedTriadNoiseEstimator<U, M, T, ?, ?> windowedNoiseEstimator,
            final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> accumulatedNoiseEstimator,
            final L listener) {
        this(windowedNoiseEstimator, accumulatedNoiseEstimator);
        mListener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return mWindowSize;
    }

    /**
     * Sets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     * Window size must always be larger than allowed minimum value, which is 2 and
     * must have and odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if detector is busy processing a previous sample.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mWindowedNoiseEstimator.setWindowSize(windowSize);
        mWindowSize = windowSize;
    }

    /**
     * Gets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return mInitialStaticSamples;
    }

    /**
     * Sets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @param initialStaticSamples number of samples to be processed initially.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_INITIAL_STATIC_SAMPLES}
     */
    public void setInitialStaticSamples(final int initialStaticSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialStaticSamples < MINIMUM_INITIAL_STATIC_SAMPLES) {
            throw new IllegalArgumentException();
        }

        mInitialStaticSamples = initialStaticSamples;
    }

    /**
     * Gets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @return factor to be applied to detected base noise level.
     */
    public double getThresholdFactor() {
        return mThresholdFactor;
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
        if (thresholdFactor <= 0.0) {
            throw new IllegalArgumentException();
        }

        mThresholdFactor = thresholdFactor;
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
        return mInstantaneousNoiseLevelFactor;
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
        if (instantaneousNoiseLevelFactor <= 0.0) {
            throw new IllegalArgumentException();
        }

        mInstantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor;
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     *
     * @return overall absolute threshold to determine whether there has
     * been excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return mBaseNoiseLevelAbsoluteThreshold;
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
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
        if (baseNoiseLevelAbsoluteThreshold <= 0.0) {
            throw new IllegalArgumentException();
        }

        mBaseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold;
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
    public M getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return createMeasurement(mBaseNoiseLevelAbsoluteThreshold, getDefaultUnit());
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(final M result) {
        result.setValue(mBaseNoiseLevelAbsoluteThreshold);
        result.setUnit(getDefaultUnit());
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
            final M baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        setBaseNoiseLevelAbsoluteThreshold(
                convertMeasurement(baseNoiseLevelAbsoluteThreshold));
    }

    /**
     * Gets listener to handle events generated by this detector.
     *
     * @return listener to handle events.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events generated by this detector.
     *
     * @param listener listener to handle events.
     * @throws LockedException if detector is busy.
     */
    public void setListener(final L listener) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets time interval between triad samples expressed in seconds (s).
     *
     * @return time interval between triad samples.
     */
    public double getTimeInterval() {
        return mWindowedNoiseEstimator.getTimeInterval();
    }

    /**
     * Sets time interval between triad samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between triad samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        mWindowedNoiseEstimator.setTimeInterval(timeInterval);
        mAccumulatedNoiseEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between triad samples.
     *
     * @return time interval between triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(getTimeInterval(), TimeUnit.SECOND);
    }

    /**
     * Gets time interval between triad samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(getTimeInterval());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between triad samples.
     *
     * @param timeInterval time interval between triad samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets current status of this detector.
     *
     * @return current status of this detector.
     */
    public Status getStatus() {
        return mStatus;
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or
     * Teslas (T) for magnetic flux density.
     *
     * @return base noise level.
     */
    public double getBaseNoiseLevel() {
        return mBaseNoiseLevel;
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization.
     *
     * @return measurement base noise level.
     */
    public M getBaseNoiseLevelAsMeasurement() {
        return createMeasurement(mBaseNoiseLevel, getDefaultUnit());
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAsMeasurement(final M result) {
        result.setValue(mBaseNoiseLevel);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in
     * (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement base noise level PSD.
     */
    public double getBaseNoiseLevelPsd() {
        return mBaseNoiseLevel * mBaseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets measurement base noise level root PSD (Power SpectralDensity) expressed
     * in (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5)
     * for magnetometer.
     *
     * @return measurement base noise level root PSD.
     */
    public double getBaseNoiseLevelRootPsd() {
        return mBaseNoiseLevel * Math.sqrt(getTimeInterval());
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for acceleration, radians per second
     * (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public M getThresholdAsMeasurement() {
        return createMeasurement(mThreshold, getDefaultUnit());
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final M result) {
        result.setValue(mThreshold);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Indicates whether this detector is busy processing last provided sample.
     *
     * @return true if this detector is busy, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getProcessedSamples() {
        return mProcessedSamples;
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average x-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgX() {
        return mAccumulatedAvgX;
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average x-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgXAsMeasurement() {
        return createMeasurement(mAccumulatedAvgX, getDefaultUnit());
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgXAsMeasurement(final M result) {
        result.setValue(mAccumulatedAvgX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgY() {
        return mAccumulatedAvgY;
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgYAsMeasurement() {
        return createMeasurement(mAccumulatedAvgY, getDefaultUnit());
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgYAsMeasurement(final M result) {
        result.setValue(mAccumulatedAvgY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgZ() {
        return mAccumulatedAvgZ;
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average z-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgZAsMeasurement() {
        return createMeasurement(mAccumulatedAvgZ, getDefaultUnit());
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgZAsMeasurement(final M result) {
        result.setValue(mAccumulatedAvgZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average measurements triad during last static period.
     */
    public T getAccumulatedAvgTriad() {
        return createTriad(mAccumulatedAvgX, mAccumulatedAvgY, mAccumulatedAvgZ, getDefaultUnit());
    }

    /**
     * Gets average measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgTriad(final T result) {
        result.setValueCoordinatesAndUnit(mAccumulatedAvgX, mAccumulatedAvgY, mAccumulatedAvgZ,
                getDefaultUnit());
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of x-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdX() {
        return mAccumulatedStdX;
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of x-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdXAsMeasurement() {
        return createMeasurement(mAccumulatedStdX, getDefaultUnit());
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdXAsMeasurement(final M result) {
        result.setValue(mAccumulatedStdX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of y-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdY() {
        return mAccumulatedStdY;
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of y-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdYAsMeasurement() {
        return createMeasurement(mAccumulatedStdY, getDefaultUnit());
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdYAsMeasurement(final M result) {
        result.setValue(mAccumulatedStdY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of z-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdZ() {
        return mAccumulatedStdZ;
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of z-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdZAsMeasurement() {
        return createMeasurement(mAccumulatedStdZ, getDefaultUnit());
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdZAsMeasurement(final M result) {
        result.setValue(mAccumulatedStdZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of measurements accumulated during last static
     * period.
     */
    public T getAccumulatedStdTriad() {
        return createTriad(mAccumulatedStdX, mAccumulatedStdY, mAccumulatedStdZ,
                getDefaultUnit());
    }

    /**
     * Gets standard deviation of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdTriad(final T result) {
        result.setValueCoordinatesAndUnit(
                mAccumulatedStdX, mAccumulatedStdY, mAccumulatedStdZ,
                getDefaultUnit());
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average x-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgX() {
        return mInstantaneousAvgX;
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average x-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgXAsMeasurement() {
        return createMeasurement(mInstantaneousAvgX, getDefaultUnit());
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgXAsMeasurement(final M result) {
        result.setValue(mInstantaneousAvgX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average y-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgY() {
        return mInstantaneousAvgY;
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average y-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgYAsMeasurement() {
        return createMeasurement(mInstantaneousAvgY, getDefaultUnit());
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgYAsMeasurement(final M result) {
        result.setValue(mInstantaneousAvgY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average z-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgZ() {
        return mInstantaneousAvgZ;
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average z-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgZAsMeasurement() {
        return createMeasurement(mInstantaneousAvgZ, getDefaultUnit());
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgZAsMeasurement(final M result) {
        result.setValue(mInstantaneousAvgZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average of measurements for each processed triad.
     */
    public T getInstantaneousAvgTriad() {
        return createTriad(
                mInstantaneousAvgX, mInstantaneousAvgY, mInstantaneousAvgZ,
                getDefaultUnit());
    }

    /**
     * Gets windowed average of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgTriad(final T result) {
        result.setValueCoordinatesAndUnit(
                mInstantaneousAvgX, mInstantaneousAvgY, mInstantaneousAvgZ,
                getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of x-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdX() {
        return mInstantaneousStdX;
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of x-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdXAsMeasurement() {
        return createMeasurement(mInstantaneousStdX, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdXAsMeasurement(final M result) {
        result.setValue(mInstantaneousStdX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of y-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdY() {
        return mInstantaneousStdY;
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation of the samples within the window.
     *
     * @return windowed standard deviation of y-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdYAsMeasurement() {
        return createMeasurement(mInstantaneousStdY, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation of the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdYAsMeasurement(final M result) {
        result.setValue(mInstantaneousStdY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of z-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdZ() {
        return mInstantaneousStdZ;
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of z-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdZAsMeasurement() {
        return createMeasurement(mInstantaneousStdZ, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdZAsMeasurement(final M result) {
        result.setValue(mInstantaneousStdZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of measurements for each processed
     * triad.
     */
    public T getInstantaneousStdTriad() {
        return createTriad(
                mInstantaneousStdX, mInstantaneousStdY, mInstantaneousStdZ,
                getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdTriad(final T result) {
        result.setValueCoordinatesAndUnit(
                mInstantaneousStdX, mInstantaneousStdY, mInstantaneousStdZ,
                getDefaultUnit());
    }

    /**
     * Processes a new measurement triad sample.
     *
     * @param triad a new measurement triad to be processed.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final T triad) throws LockedException {
        return process(convertMeasurement(triad.getValueX(), triad.getUnit()),
                convertMeasurement(triad.getValueY(), triad.getUnit()),
                convertMeasurement(triad.getValueZ(), triad.getUnit()));
    }

    /**
     * Processes a new measurement triad sample.
     *
     * @param valueX x-coordinate of sensed measurement.
     * @param valueY y-coordinate of sensed measurement.
     * @param valueZ z-coordinate of sensed measurement.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final M valueX, final M valueY, final M valueZ)
            throws LockedException {
        return process(convertMeasurement(valueX),
                convertMeasurement(valueY),
                convertMeasurement(valueZ));
    }

    /**
     * Processes a new measurement triad sample.
     * Provided measurement coordinates are expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     *
     * @param valueX x-coordinate of sensed measurement.
     * @param valueY y-coordinate of sensed measurement.
     * @param valueZ z-coordinate of sensed measurement.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final double valueX,
                           final double valueY,
                           final double valueZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mStatus == Status.FAILED) {
            return false;
        }

        mRunning = true;

        if (mStatus == Status.IDLE) {
            // start initialization
            mStatus = Status.INITIALIZING;

            if (mListener != null) {
                //noinspection unchecked
                mListener.onInitializationStarted((D) this);
            }
        }

        mProcessedSamples++;

        mWindowedNoiseEstimator.addTriadAndProcess(valueX, valueY, valueZ);

        mInstantaneousAvgX = mWindowedNoiseEstimator.getAvgX();
        mInstantaneousAvgY = mWindowedNoiseEstimator.getAvgY();
        mInstantaneousAvgZ = mWindowedNoiseEstimator.getAvgZ();

        mInstantaneousStdX = mWindowedNoiseEstimator.getStandardDeviationX();
        mInstantaneousStdY = mWindowedNoiseEstimator.getStandardDeviationY();
        mInstantaneousStdZ = mWindowedNoiseEstimator.getStandardDeviationZ();

        final double windowedStdNorm = mWindowedNoiseEstimator
                .getStandardDeviationNorm();

        final boolean filledWindow = mWindowedNoiseEstimator.isWindowFilled();

        if (mStatus == Status.INITIALIZING) {
            // process sample during initialization
            mAccumulatedNoiseEstimator.addTriad(valueX, valueY, valueZ);
            final double accumulatedStdNorm = mAccumulatedNoiseEstimator
                    .getStandardDeviationNorm();

            if (mProcessedSamples < mInitialStaticSamples) {
                if (filledWindow && (windowedStdNorm / accumulatedStdNorm > mInstantaneousNoiseLevelFactor)) {
                    // sudden motion detected
                    mStatus = Status.FAILED;

                    // notify error
                    if (mListener != null) {
                        //noinspection unchecked
                        mListener.onError((D) this, accumulatedStdNorm,
                                windowedStdNorm,
                                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED);
                    }
                }

            } else if (filledWindow) {
                // initialization completed
                // set base noise level and threshold
                mBaseNoiseLevel = accumulatedStdNorm;
                mThreshold = mBaseNoiseLevel * mThresholdFactor;

                // keep average/std measurements triad in case we want to obtain
                // its value since initial period must be static
                mAccumulatedAvgX = mAccumulatedNoiseEstimator.getAvgX();
                mAccumulatedAvgY = mAccumulatedNoiseEstimator.getAvgY();
                mAccumulatedAvgZ = mAccumulatedNoiseEstimator.getAvgZ();

                mAccumulatedStdX = mAccumulatedNoiseEstimator.getStandardDeviationX();
                mAccumulatedStdY = mAccumulatedNoiseEstimator.getStandardDeviationY();
                mAccumulatedStdZ = mAccumulatedNoiseEstimator.getStandardDeviationZ();

                // reset accumulated estimator so that we can estimate
                // average specific force in static periods
                mAccumulatedNoiseEstimator.reset();

                if (mBaseNoiseLevel > mBaseNoiseLevelAbsoluteThreshold) {
                    // base noise level exceeds allowed value
                    mStatus = Status.FAILED;

                    // notify error
                    if (mListener != null) {
                        //noinspection unchecked
                        mListener.onError((D) this, accumulatedStdNorm,
                                windowedStdNorm,
                                ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED);
                    }

                } else {
                    // initialization has been successfully completed
                    mStatus = Status.INITIALIZATION_COMPLETED;

                    if (mListener != null) {
                        //noinspection unchecked
                        mListener.onInitializationCompleted(
                                (D) this, mBaseNoiseLevel);
                    }
                }

            }

            mRunning = false;
            return true;
        } else {
            // detect static or dynamic period
            final Status previousStatus = mStatus;

            if (windowedStdNorm < mThreshold) {
                mStatus = Status.STATIC_INTERVAL;
            } else {
                mStatus = Status.DYNAMIC_INTERVAL;
            }

            if (mStatus == Status.STATIC_INTERVAL) {
                // while we are in static interval, keep adding samples to estimate
                // accumulated average measurement triad
                mAccumulatedNoiseEstimator.addTriad(valueX, valueY, valueZ);
            }

            if (previousStatus != mStatus) {
                // static/dynamic period change detected
                switch (mStatus) {
                    case STATIC_INTERVAL:
                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onStaticIntervalDetected((D) this,
                                    mInstantaneousAvgX, mInstantaneousAvgY, mInstantaneousAvgZ,
                                    mInstantaneousStdX, mInstantaneousStdY, mInstantaneousStdZ);
                        }
                        break;
                    case DYNAMIC_INTERVAL:
                        // when switching from static to dynamic interval,
                        // pick accumulated average and standard deviation measurement triads
                        mAccumulatedAvgX = mAccumulatedNoiseEstimator.getAvgX();
                        mAccumulatedAvgY = mAccumulatedNoiseEstimator.getAvgY();
                        mAccumulatedAvgZ = mAccumulatedNoiseEstimator.getAvgZ();

                        mAccumulatedStdX = mAccumulatedNoiseEstimator.getStandardDeviationX();
                        mAccumulatedStdY = mAccumulatedNoiseEstimator.getStandardDeviationY();
                        mAccumulatedStdZ = mAccumulatedNoiseEstimator.getStandardDeviationZ();

                        // reset accumulated estimator when switching to dynamic period
                        mAccumulatedNoiseEstimator.reset();

                        if (mListener != null) {
                            //noinspection unchecked
                            mListener.onDynamicIntervalDetected((D) this,
                                    mInstantaneousAvgX, mInstantaneousAvgY, mInstantaneousAvgZ,
                                    mInstantaneousStdX, mInstantaneousStdY, mInstantaneousStdZ,
                                    mAccumulatedAvgX, mAccumulatedAvgY, mAccumulatedAvgZ,
                                    mAccumulatedStdX, mAccumulatedStdY, mAccumulatedStdZ);
                        }
                        break;
                }
            }
        }

        mRunning = false;
        return true;
    }

    /**
     * Resets this detector so that it is initialized again when new samples are added.
     *
     * @throws LockedException if detector is busy.
     */
    public void reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;

        mStatus = Status.IDLE;
        mProcessedSamples = 0;
        mBaseNoiseLevel = 0.0;
        mThreshold = 0.0;

        mWindowedNoiseEstimator.reset();
        mAccumulatedNoiseEstimator.reset();

        if (mListener != null) {
            //noinspection unchecked
            mListener.onReset((D) this);
        }

        mRunning = false;
    }

    /**
     * Converts provided measurement instance to its default unit, which is
     * meters per squared second (m/s^2) for acceleration, radians per second (rad/s) for
     * angular speed or Teslas (T) for magnetic flux density.
     *
     * @param measurement measurement to be converted.
     * @return converted value.
     */
    protected double convertMeasurement(M measurement) {
        return convertMeasurement(measurement.getValue().doubleValue(),
                measurement.getUnit());
    }

    /**
     * Converts provided measurement value expressed in provided unit to the
     * default measurement value, which is meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (t) for magnetic flux density.
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    protected abstract double convertMeasurement(final double value, final U unit);

    /**
     * Creates a measurement instance using provided value and unit.
     *
     * @param value value of measurement.
     * @param unit  unit of value.
     * @return created measurement
     */
    protected abstract M createMeasurement(final double value, final U unit);

    /**
     * Gets default unit for measurements this implementation works with.
     *
     * @return default measurement unit.
     */
    protected abstract U getDefaultUnit();

    /**
     * Creates a triad.
     *
     * @param valueX x-coordinate value.
     * @param valueY y-coordinate value.
     * @param valueZ z-coordinate value.
     * @param unit   unit of values.
     * @return created triad.
     */
    protected abstract T createTriad(
            final double valueX, final double valueY, final double valueZ, final U unit);

    /**
     * Possible detector status values.
     */
    public enum Status {
        /**
         * Detector is in idle status when it hasn't processed any sample yet.
         */
        IDLE,

        /**
         * Detector is processing samples in the initial static process to determine base noise level.
         */
        INITIALIZING,

        /**
         * Detector has sucessfully completed processing samples on the initial
         * static period.
         */
        INITIALIZATION_COMPLETED,

        /**
         * A static interval has been detected, where accelerometer is considered to be subject to no substantial
         * movement forces.
         */
        STATIC_INTERVAL,

        /**
         * A dynamic interval has been detected, where accelerometer is considered to be subject to substantial
         * movement forces.
         */
        DYNAMIC_INTERVAL,

        /**
         * Detector has failed. This happens if accelerometer is subject to sudden movement forces while detector
         * is initializing during the initial static period.
         * When detector has failed, no new samples will be allowed to be processed until detector is reset.
         */
        FAILED
    }

    /**
     * Reason why this detector has failed during initialization.
     */
    public enum ErrorReason {
        /**
         * If a sudden movement is detected during initialization.
         */
        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,

        /**
         * If overall noise level is excessive during initialization.
         */
        OVERALL_EXCESSIVE_MOVEMENT_DETECTED
    }
}
