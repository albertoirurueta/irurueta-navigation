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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.io.IOException;
import java.util.List;

/**
 * Robustly estimates magnetometer soft-iron cross
 * couplings and scaling factors using LMedS algorithm.
 * <p>
 * To use this calibrator at least 4 measurements at different known
 * frames must be provided. In other words, magnetometer samples must
 * be obtained at 4 different positions or orientations.
 * Notice that frame velocities are ignored by this calibrator.
 * <p>
 * Measured magnetic flux density is assumed to follow the model shown below:
 * <pre>
 *     mBmeas = bm + (I + Mm) * mBtrue + w
 * </pre>
 * Where:
 * - mBmeas is the measured magnetic flux density. This is a 3x1 vector.
 * - bm is magnetometer hard-iron bias. Ideally, on a perfect magnetometer,
 * this should be a 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mm is the 3x3 soft-iron matrix containing cross-couplings and scaling
 * factors. Ideally, on a perfect magnetometer, this should be a 3x3 zero
 * matrix.
 * - mBtrue is ground-truth magnetic flux density. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public class LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator extends
        RobustKnownHardIronAndFrameMagnetometerCalibrator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in LMedS, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 500e-9;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in LMedS, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Constructor.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        super(commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algrithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in LMedS, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in LMedS, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setStopThreshold(double stopThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = stopThreshold;
    }

    /**
     * Estimates magnetometer calibration parameters containing
     * soft-iron scale factors and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException,
            CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final LMedSRobustEstimator<Matrix> innerEstimator =
                new LMedSRobustEstimator<>(new LMedSRobustEstimatorListener<Matrix>() {
                    @Override
                    public int getTotalSamples() {
                        return mMeasurements.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return mPreliminarySubsetSize;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices,
                            final List<Matrix> solutions) {
                        computePreliminarySolutions(samplesIndices,
                                solutions);
                    }

                    @Override
                    public double computeResidual(
                            final Matrix currentEstimation,
                            final int i) {
                        return computeError(mMeasurements.get(i),
                                currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(
                            final RobustEstimator<Matrix> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateStart(
                                    LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(
                            final RobustEstimator<Matrix> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateEnd(
                                    LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<Matrix> estimator,
                            final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<Matrix> estimator,
                            final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.this,
                                    progress);
                        }
                    }
                });

        try {
            mRunning = true;
            mInliersData = null;

            setupWmmEstimator();

            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
            final Matrix preliminaryResult = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (RobustEstimatorException | IOException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.LMedS;
    }
}
