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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.LockedException;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;


/**
 * Estimates average time interval between processed samples.
 */
public class IMUTimeIntervalEstimator {

    /**
     * Default total samples to be processed.
     */
    public static final int DEFAULT_TOTAL_SAMPLES = 100000;

    /**
     * Total samples to be processed to finish estimation.
     */
    private int mTotalSamples = DEFAULT_TOTAL_SAMPLES;

    /**
     * Listener to handle events raised by this estimator.
     */
    private IMUTimeIntervalEstimatorListener mListener;

    /**
     * Last provided timestamp expressed in seconds (s).
     */
    private Double mLastTimestamp;

    /**
     * Estimated average time interval between body kinematics samples expressed in
     * seconds (s).
     */
    private double mAverageTimeInterval;

    /**
     * Estimated variance of time interval between body kinematics samples expressed
     * in squared seconds (s^2).
     */
    private double mTimeIntervalVariance;

    /**
     * Number of processed timestamp samples.
     */
    private int mNumberOfProcessedSamples;

    /**
     * Number of processed timestamp samples plus one.
     */
    private int mNumberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public IMUTimeIntervalEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public IMUTimeIntervalEstimator(final IMUTimeIntervalEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUTimeIntervalEstimator(final int totalSamples) {
        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUTimeIntervalEstimator(final int totalSamples,
                                    final IMUTimeIntervalEstimatorListener listener) {
        this(totalSamples);
        mListener = listener;
    }

    /**
     * Gets total samples to be processed to finish estimation.
     *
     * @return total samples to be processed to finish estimation.
     */
    public int getTotalSamples() {
        return mTotalSamples;
    }

    /**
     * Sets total samples to be processed to finish estimation.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws LockedException if estimator is currently running.
     */
    public void setTotalSamples(final int totalSamples) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public IMUTimeIntervalEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final IMUTimeIntervalEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets last provided timestamp expressed in seconds or null if none has been
     * provided yet.
     *
     * @return last provided timestamp or null.
     */
    public Double getLastTimestamp() {
        return mLastTimestamp;
    }

    /**
     * Gets last provided timestamp or null if none has been provided yet.
     *
     * @return last provided timestamp or null.
     */
    public Time getLastTimestampAsTime() {
        return mLastTimestamp != null ?
                new Time(mLastTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Gets last provided timestamp.
     *
     * @param result instance where last provided timestamp will be stored.
     * @return true if last provided timestamp was available, false otherwise.
     */
    public boolean getLastTimestampAsTime(final Time result) {
        if (mLastTimestamp != null) {
            result.setValue(mLastTimestamp);
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated average time interval between body kinematics samples expressed in
     * seconds (s).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated average time interval.
     */
    public double getAverageTimeInterval() {
        return mAverageTimeInterval;
    }

    /**
     * Gets estimated average time interval between body kinematics samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimate average time interval.
     */
    public Time getAverageTimeIntervalAsTime() {
        return new Time(mAverageTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets estimated average time interval between body kinematics samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @param result instance where estimated average time interval will be stored.
     */
    public void getAverageTimeIntervalAsTime(final Time result) {
        result.setValue(mAverageTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets estimated variance of time interval between body kinematics samples
     * expressed in squared seconds (s^2).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated variance of time interval between body kinematics samples.
     */
    public double getTimeIntervalVariance() {
        return mTimeIntervalVariance;
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples expressed in seconds (s).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated standard deviation of time interval between body kinematics
     * samples.
     */
    public double getTimeIntervalStandardDeviation() {
        return Math.sqrt(mTimeIntervalVariance);
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated standard deviation of time interval between body kinematics
     * samples.
     */
    public Time getTimeIntervalStandardDeviationAsTime() {
        return new Time(getTimeIntervalStandardDeviation(), TimeUnit.SECOND);
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @param result instance where estimated standard deviation of time interval
     *               between body kinematics samples will be stored.
     */
    public void getTimeIntervalStandardDeviationAsTime(final Time result) {
        result.setValue(getTimeIntervalStandardDeviation());
        result.setUnit(TimeUnit.SECOND);
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
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether estimator has finished the estimation.
     *
     * @return true if estimator has finished, false otherwise.
     */
    public boolean isFinished() {
        return mNumberOfProcessedSamples == mTotalSamples;
    }

    /**
     * Adds a timestamp value to current estimation.
     * If estimator is already finished, provided timestamp will be ignored.
     *
     * @param timestamp timestamp since epoch time.
     * @return true if provided timestamp has been processed, false if it has been
     * ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addTimestamp(final Time timestamp) throws LockedException {
        return addTimestamp(TimeConverter.convert(timestamp.getValue().doubleValue(),
                timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Adds a timestamp value to current estimation.
     * If estimator is already finished, provided timestamp will be ignored.
     *
     * @param timestamp timestamp since epoch time expressed in seconds (s).
     * @return true if provided timestamp has been processed, false if it has been
     * ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addTimestamp(final double timestamp) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (isFinished()) {
            return false;
        }

        mRunning = true;

        if (mLastTimestamp == null && mListener != null) {
            mListener.onStart(this);
        }

        if (mLastTimestamp != null) {
            final double timeInterval = timestamp - mLastTimestamp;

            mAverageTimeInterval = mAverageTimeInterval * (double) mNumberOfProcessedSamples
                    / (double) mNumberOfProcessedSamplesPlusOne
                    + timeInterval / (double) mNumberOfProcessedSamplesPlusOne;

            final double diff = timeInterval - mAverageTimeInterval;
            final double diff2 = diff * diff;
            mTimeIntervalVariance = mTimeIntervalVariance * (double) mNumberOfProcessedSamples
                    / (double) mNumberOfProcessedSamplesPlusOne
                    + diff2 / (double) mNumberOfProcessedSamplesPlusOne;
        }

        mLastTimestamp = timestamp;

        mNumberOfProcessedSamples++;
        mNumberOfProcessedSamplesPlusOne++;

        if (mListener != null) {
            mListener.onTimestampAdded(this);
        }

        mRunning = false;

        if (isFinished() && mListener != null) {
            mListener.onFinish(this);
        }

        return true;
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

        mRunning = true;
        mLastTimestamp = null;
        mAverageTimeInterval = 0.0;
        mTimeIntervalVariance = 0.0;
        mNumberOfProcessedSamples = 0;
        mNumberOfProcessedSamplesPlusOne = 1;

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;

        return true;
    }
}
