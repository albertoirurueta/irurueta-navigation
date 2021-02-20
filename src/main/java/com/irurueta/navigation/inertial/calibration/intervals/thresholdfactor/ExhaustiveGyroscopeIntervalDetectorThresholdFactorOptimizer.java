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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;

/**
 * Optimizes threshold factor for interval detection of accelerometer + gyroscope data
 * based on results of calibration.
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 * This implementation makes an exhaustive search between minimum and maximum
 * threshold factor values in order to find the threshold value that produces the
 * minimum Mean Square Error (MSE) for calibration parameters.
 */
public class ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer extends
        GyroscopeIntervalDetectorThresholdFactorOptimizer {

    /**
     * Default step value to make exhaustive search of threshold factor values.
     */
    public static final double DEFAULT_STEP = 1.0;

    /**
     * Step to make exhaustive search of threshold factor values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     */
    private double mThresholdFactorStep = DEFAULT_STEP;

    /**
     * Constructor.
     */
    public ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer() {
        super();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
    }

    /**
     * Constructor.
     *
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeNonLinearCalibrator calibrator) {
        super(calibrator);
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final GyroscopeNonLinearCalibrator calibrator) {
        super(dataSource, calibrator);
    }

    /**
     * Gets step to make exhaustive search of threshold values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     *
     * @return step to make exhaustive search of threshold values.
     */
    public double getThresholdFactorStep() {
        return mThresholdFactorStep;
    }

    /**
     * Sets step to make exhaustive search of threshold values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     *
     * @param thresholdStep step to make exhaustive search of threshold values.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setThresholdFactorStep(final double thresholdStep)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (thresholdStep <= 0.0) {
            throw new IllegalArgumentException();
        }

        mThresholdFactorStep = thresholdStep;
    }

    /**
     * Optimizes threshold factor for a static interval detector or measurement
     * generator in order to minimize MSE (Minimum Squared Error) of estimated
     * calibration parameters.
     *
     * @return optimized threshold factor.
     * @throws NotReadyException                                 if this optimizer is not ready to start optimization.
     * @throws LockedException                                   if optimizer is already running.
     * @throws IntervalDetectorThresholdFactorOptimizerException if optimization fails for
     *                                                           some reason.
     */
    @Override
    public double optimize() throws NotReadyException, LockedException,
            IntervalDetectorThresholdFactorOptimizerException {
        if (mRunning) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        boolean hasResult = false;
        mMinMse = Double.MAX_VALUE;
        try {
            mRunning = true;

            initProgress();
            final float progressStep = (float) (mThresholdFactorStep
                    / (mThresholdFactorStep + mMaxThresholdFactor - mMinThresholdFactor));

            if (mListener != null) {
                mListener.onOptimizeStart(this);
            }

            for (double thresholdFactor = mMinThresholdFactor;
                 thresholdFactor <= mMaxThresholdFactor;
                 thresholdFactor += mThresholdFactorStep) {
                try {
                    evaluateForThresholdFactor(thresholdFactor);
                    hasResult = true;
                } catch (final Exception ignore) {
                    // when an error occurs, skip to next iteration
                }

                mProgress += progressStep;
                checkAndNotifyProgress();
            }

            if (!hasResult) {
                throw new IntervalDetectorThresholdFactorOptimizerException();
            }

            if (mListener != null) {
                mListener.onOptimizeEnd(this);
            }

        } finally {
            mRunning = false;
        }

        return mOptimalThresholdFactor;
    }
}
