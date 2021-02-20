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

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Optimizes threshold factor for interval detection of accelerometer data based
 * on results of calibration of accelerometers, gyroscopes or magnetometers.
 *
 * @param <T> type of data to be used as input for this optimizer.
 * @param <S> type of data source for this optimizer.
 */
public abstract class IntervalDetectorThresholdFactorOptimizer<T,
        S extends IntervalDetectorThresholdFactorOptimizerDataSource<T>> {

    /**
     * Default amount of progress variation before notifying a change in optimization progress.
     * By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Retrieves data for this optimizer.
     */
    protected S mDataSource;

    /**
     * Indicates whether this optimizer is running or not.
     */
    protected boolean mRunning;

    /**
     * Minimum Mean Square Error that has been found.
     */
    protected double mMinMse;

    /**
     * Optimal threshold factor that has been found.
     */
    protected double mOptimalThresholdFactor;

    /**
     * Listener that notifies events generated by this optimizer.
     */
    protected IntervalDetectorThresholdFactorOptimizerListener<T, S> mListener;

    /**
     * Amount of progress variation before notifying a progress change during optimization.
     */
    protected float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Current progress of optimization.
     */
    protected float mProgress;

    /**
     * Previously notified progress of optimization.
     */
    protected float mPreviousProgress;

    /**
     * Constructor.
     */
    public IntervalDetectorThresholdFactorOptimizer() {
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public IntervalDetectorThresholdFactorOptimizer(final S dataSource) {
        mDataSource = dataSource;
    }

    /**
     * Gets instance in charge of retrieving data for this optimizer.
     *
     * @return instance in charge of retrieving data for this optimizer.
     */
    public S getDataSource() {
        return mDataSource;
    }

    /**
     * Sets instance in charge of retrieving data for this optimizer.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     * @throws LockedException if optimizer is already running.
     */
    public void setDataSource(
            final S dataSource)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mDataSource = dataSource;
    }

    /**
     * Gets listener that notifies events generated by this optimizer.
     *
     * @return listener that notifies events generated by this optimizer.
     */
    public IntervalDetectorThresholdFactorOptimizerListener<T, S> getListener() {
        return mListener;
    }

    /**
     * Sets listener that notifies events generated by this optimizer.
     *
     * @param listener listener that notifies events generated by this optimizer.
     * @throws LockedException if optimizer is already running.
     */
    public void setListener(
            final IntervalDetectorThresholdFactorOptimizerListener<T, S> listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * optimization.
     *
     * @return amount of progress variation before notifying a progress change during
     * optimization.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * optimization.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during optimization.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if optimizer is currently running.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Indicates whether this optimizer is busy optimizing a threshold factor.
     *
     * @return true if optimizer is busy, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether this optimizer is ready to start optimization.
     *
     * @return true if this optimizer is ready, false otherwise.
     */
    public boolean isReady() {
        return mDataSource != null;
    }

    /**
     * Gets minimum Mean Square Error that has been found for calibration.
     *
     * @return minimum Mean Square Error that has been found.
     */
    public double getMinMse() {
        return mMinMse;
    }

    /**
     * Gets optimal threshold factor that has been found.
     *
     * @return optimal threshold factor that has been found.
     */
    public double getOptimalThresholdFactor() {
        return mOptimalThresholdFactor;
    }

    /**
     * Optimizes threshold factor for a static interval detector or measurement
     * generator in order to minimize MSE (Minimum Squared Error) of estimated
     * calibration parameters.
     *
     * @return optimized threshold factor.
     * @throws NotReadyException                           if this optimizer is not
     *                                                     ready to start optimization.
     * @throws LockedException                             if optimizer is already
     *                                                     running.
     * @throws IntervalDetectorThresholdFactorOptimizerException if optimization fails for
     *                                                     some reason.
     */
    public abstract double optimize()
            throws NotReadyException, LockedException,
            IntervalDetectorThresholdFactorOptimizerException;

    /**
     * Checks current progress and notifies if progress has changed significantly.
     */
    protected void checkAndNotifyProgress() {
        if (mListener != null) {
            if (mProgress - mPreviousProgress > mProgressDelta) {
                mPreviousProgress = mProgress;
                mListener.onOptimizeProgressChange(this, Math.min(mProgress, 1.0f));
            }
        }
    }

    /**
     * Initializes progress values.
     */
    protected void initProgress() {
        mPreviousProgress = 0.0f;
        mProgress = 0.0f;
    }
}
