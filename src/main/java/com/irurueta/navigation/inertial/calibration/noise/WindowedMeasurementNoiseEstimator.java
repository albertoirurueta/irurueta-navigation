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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.units.Measurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.Arrays;

/**
 * Base class to estimate measurement noise variances and PSD's (Power Spectral Densities)
 * along with average values for a windowed amount of samples.
 * Implementations of this estimator may use norms of measurement triads to estimate noise
 * levels.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * Notice that if there are less than {@link #getWindowSize()} processed
 * samples in the window, this estimator will assume that the remaining ones
 * until the window is completed have zero values.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <E> an estimator type.
 * @param <L> a listener type.
 */
public abstract class WindowedMeasurementNoiseEstimator<U extends Enum<?>,
        M extends Measurement<U>, E extends WindowedMeasurementNoiseEstimator<U, M, E, L>,
        L extends WindowedMeasurementNoiseEstimatorListener<U, M, E>> {

    /**
     * Number of samples to keep within the window by default.
     * For an accelerometer generating 100 samples/second, this is equivalent to
     * 1 second.
     * For an accelerometer generating 50 samples/second, this is equivalent to
     * 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = 101;

    /**
     * Minimum allowed window size.
     */
    public static final int MIN_WINDOW_SIZE = 3;

    /**
     * Default time interval between accelerometer samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must
     * have and odd value.
     */
    private int mWindowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Time interval expressed in seconds (s) between consecutive measurements.
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Keeps the window of measurements expressed in their default units.
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double[] mWindowedMeasurements = new double[DEFAULT_WINDOW_SIZE];

    /**
     * Listener to handle events raised by this estimator.
     */
    private L mListener;

    /**
     * Contains estimated average of measurement expressed in its default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double mAvg;

    /**
     * Contains estimated variance of measurement expressed in its default squared unit
     * (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic
     * flux density).
     */
    private double mVariance;

    /**
     * Indicates popsition of first element in window.
     */
    private int mFirstCursor;

    /**
     * Indicates position of last element in window.
     */
    private int mLastCursor;

    /**
     * Number of processed measurement samples.
     */
    private int mNumberOfProcessedSamples;

    /**
     * Number of added measurement samples.
     */
    private int mNumberOfAddedSamples;

    /**
     * Indicates whether estimator is running or not.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public WindowedMeasurementNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public WindowedMeasurementNoiseEstimator(final L listener) {
        mListener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must have
     * and odd value.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return mWindowSize;
    }

    /**
     * Sets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must have
     * an odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws LockedException          if estimator is currently running.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        // check that window is larger than minimum allowed value
        if (windowSize < MIN_WINDOW_SIZE) {
            throw new IllegalArgumentException();
        }

        // check that window size is not even
        if (windowSize % 2 == 0) {
            throw new IllegalArgumentException();
        }

        mWindowSize = windowSize;
        mWindowedMeasurements = new double[windowSize];
        reset();
    }

    /**
     * Gets time interval between accelerometer triad samples expressed in
     * seconds (s).
     *
     * @return time interval between accelerometer triad samples.
     */
    public double getTimeInterval() {
        return mTimeInterval;
    }

    /**
     * Sets time interval between accelerometer triad samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between accelerometer triad samples.
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

        mTimeInterval = timeInterval;
    }

    /**
     * Gets time interval between accelerometer triad samples.
     *
     * @return time interval between accelerometer triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(mTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between accelerometer triad samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(mTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between accelerometer triad samples.
     *
     * @param timeInterval time interval between accelerometer triad samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final L listener) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets first provided measurement value expressed in its default units.
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     *
     * @return first provided measurement value or null if not available.
     */
    public Double getFirstWindowedMeasurementValue() {
        if (mNumberOfAddedSamples == 0) {
            return null;
        } else {
            return mWindowedMeasurements[mFirstCursor % mWindowSize];
        }
    }

    /**
     * Gets las provided measurement value expressed in its default units.
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     *
     * @return last provided measurement value or null if not available.
     */
    public Double getLastWindowedMeasurementValue() {
        if (mNumberOfAddedSamples == 0) {
            return null;
        } else {
            return mWindowedMeasurements[(mLastCursor - 1 + mWindowSize) % mWindowSize];
        }
    }

    /**
     * Gets first provided measurement within the window.
     *
     * @return first provided measurement within the window or null if not available.
     */
    public M getFirstWindowedMeasurement() {
        final Double value = getFirstWindowedMeasurementValue();
        return value != null ? createMeasurement(value, getDefaultUnit()) : null;
    }

    /**
     * Gets first provided measurement within the window.
     *
     * @param result instance where first provided measurement will be stored.
     * @return true if result was updated, false if first measurement is not available.
     */
    public boolean getFirstWindowedMeasurement(final M result) {
        final Double value = getFirstWindowedMeasurementValue();
        if (value != null) {
            result.setValue(value);
            result.setUnit(getDefaultUnit());
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets last provided measurement within the window.
     *
     * @return last provided measurement within the window or null if not available.
     */
    public M getLastWindowedMeasurement() {
        final Double value = getLastWindowedMeasurementValue();
        return value != null ? createMeasurement(value, getDefaultUnit()) : null;
    }

    /**
     * Gets last provided measurement within the window.
     *
     * @param result instance where last provided measurement will be stored.
     * @return true if result was updated, false if last measurement is not available.
     */
    public boolean getLastWindowedMeasurement(final M result) {
        final Double value = getLastWindowedMeasurementValue();
        if (value != null) {
            result.setValue(value);
            result.setUnit(getDefaultUnit());
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated average of measurement expressed in its default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     *
     * @return average of measurement in current window.
     */
    public double getAvg() {
        return mAvg;
    }

    /**
     * Gets estimated average of measurement within current window.
     *
     * @return average of measurement in current window
     */
    public M getAvgAsMeasurement() {
        return createMeasurement(mAvg, getDefaultUnit());
    }

    /**
     * Gets estimated average of measurement within current window.
     *
     * @param result instance where average of measurement will be stored.
     */
    public void getAvgAsMeasurement(final M result) {
        result.setValue(mAvg);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated variance of measurement within current window
     * expressed in its default squared unit (m^2/s^4 for acceleration,
     * rad^2/s^2 for angular speed or T^2 for magnetic flux density).
     *
     * @return estimated variance of measurement within current window.
     */
    public double getVariance() {
        return mVariance;
    }

    /**
     * Gets estimated standard deviation of measurement within current window
     * and expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return estimated standard of measurement.
     */
    public double getStandardDeviation() {
        return Math.sqrt(mVariance);
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @return estimated standard deviation of measurement.
     */
    public M getStandardDeviationAsMeasurement() {
        return createMeasurement(getStandardDeviation(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @param result instance where estimated standard deviation of measurement
     *               will be stored.
     */
    public void getStandardDeviationAsMeasurement(final M result) {
        result.setValue(getStandardDeviation());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets measurement noise PSD (Power Spectral Density) expressed
     * in (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measureent noise PSD.
     */
    public double getPsd() {
        return mVariance * mTimeInterval;
    }

    /**
     * Gets measurement noise root PSD (Power Spectral Density) expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
     *
     * @return measurement noise root PSD.
     */
    public double getRootPsd() {
        return Math.sqrt(getPsd());
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return mNumberOfProcessedSamples;
    }

    /**
     * Gets number of samples that have been added so far.
     *
     * @return number of samples that have been added so far.
     */
    public int getNumberOfAddedSamples() {
        return mNumberOfAddedSamples;
    }

    /**
     * Gets number of currently windowed samples.
     *
     * @return number of samples within the window.
     */
    public int getNumberOfSamplesInWindow() {
        return Math.min(mNumberOfAddedSamples, mWindowSize);
    }

    /**
     * Indicates whether window of samples is filled or not.
     *
     * @return true if window is filled, false otherwise.
     */
    public boolean isWindowFilled() {
        return getNumberOfSamplesInWindow() == mWindowSize;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Adds a measurement value expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density) and processes current window.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param value value to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addMeasurementAndProcess(final double value) throws LockedException {
        internalAdd(value, true);
    }

    /**
     * Adds a measurement and processes current window.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param value value to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addMeasurementAndProcess(final M value) throws LockedException {
        internalAdd(convertToDefaultUnit(value), true);
    }

    /**
     * Adds a measurement value expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param value value to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addMeasurement(final double value) throws LockedException {
        internalAdd(value, false);
    }

    /**
     * Adds a measurement.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param value value to be added.
     * @throws LockedException if estimator is currently running
     */
    public void addMeasurement(final M value) throws LockedException {
        internalAdd(convertToDefaultUnit(value), false);
    }

    /**
     * Resets current estimator.
     *
     * @return true if estimator was successfully reset, false if no reset was needed.
     * @throws LockedException if estimator is currently running.
     */
    public boolean reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mNumberOfProcessedSamples == 0) {
            return false;
        }

        Arrays.fill(mWindowedMeasurements, 0.0);
        mFirstCursor = 0;
        mLastCursor = 0;
        mAvg = 0.0;
        mVariance = 0.0;
        mNumberOfProcessedSamples = 0;
        mNumberOfAddedSamples = 0;

        if (mListener != null) {
            //noinspection unchecked
            mListener.onReset((E) this);
        }

        return true;
    }

    /**
     * Gets default unit for a measurement.
     *
     * @return default unit for a measurement.
     */
    protected abstract U getDefaultUnit();

    /**
     * Creates a measurement with provided value and unit.
     *
     * @param value value to be set.
     * @param unit  unit to be set.
     * @return created measurement.
     */
    protected abstract M createMeasurement(final double value, final U unit);

    /**
     * Converts provided measurement into default unit.
     *
     * @param value measurement to be converted.
     * @return converted value.
     */
    protected abstract double convertToDefaultUnit(M value);

    /**
     * Internally adds a measurement value and processes current window if indicated.
     *
     * @param value measurement value to be added.
     * @param process true if window of samples must also be processed, false otherwise.
     * @throws LockedException if estimator is currently running.
     */
    private void internalAdd(final double value, final boolean process) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;

        if (mNumberOfAddedSamples == 0 && mListener != null) {
            //noinspection unchecked
            mListener.onStart((E) this);
        }

        final boolean wasFilled = isWindowFilled();
        if (wasFilled) {
            // increase first cursor
            mFirstCursor = (mFirstCursor + 1) % mWindowSize;
        }
        mWindowedMeasurements[mLastCursor] = value;
        mLastCursor = (mLastCursor + 1) % mWindowSize;
        mNumberOfAddedSamples++;

        // process window
        if (process) {
            processWindow();
        }

        mRunning = false;

        if (mListener != null) {
            //noinspection unchecked
            mListener.onMeasurementAdded((E) this);

            if (!wasFilled && isWindowFilled()) {
                //noinspection unchecked
                mListener.onWindowFilled((E) this);
            }
        }
    }

    /**
     * Processes current windowed samples.
     */
    private void processWindow() {
        mNumberOfProcessedSamples++;

        final int n = getNumberOfSamplesInWindow();

        final int endPos = Math.min(n, mWindowSize);

        // compute averages
        double avg = 0.0;
        for (int i = 0; i < endPos; i++) {
            final double value = mWindowedMeasurements[i];
            avg += value;
        }

        avg /= mWindowSize;

        // compute variances
        double var = 0.0;
        for (int i = 0; i < endPos; i++) {
            final double value = mWindowedMeasurements[i];
            final double diff = value - avg;
            final double diff2 = diff * diff;

            var += diff2;
        }

        final int m = mWindowSize - 1;

        var /= m;

        mAvg = avg;
        mVariance = var;
    }
}
