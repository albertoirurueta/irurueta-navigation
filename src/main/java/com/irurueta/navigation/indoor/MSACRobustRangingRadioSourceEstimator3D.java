/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly estimated 3D position of a radio source (e.g. WiFi
 * access point or bluetooth beacon), by discarding outliers using MSAC
 * algorithm.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class MSACRobustRangingRadioSourceEstimator3D<S extends RadioSource> extends
        RobustRangingRadioSourceEstimator3D<S> {

    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 0.1;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether samples are inliers or not when
     * testing possible estimation solutions.
     */
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Constructor.
     */
    public MSACRobustRangingRadioSourceEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustRangingRadioSourceEstimator3D(
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public MSACRobustRangingRadioSourceEstimator3D(Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustRangingRadioSourceEstimator3D(Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     * zero.
     * @throws LockedException if robust estimator is locked because an
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Robustly estimates position for a radio source.
     * @throws LockedException if instance is busy during estimation.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public void estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        MSACRobustEstimator<Solution<Point3D>> innerEstimator =
                new MSACRobustEstimator<>(
                        new MSACRobustEstimatorListener<Solution<Point3D>>() {
                    @Override
                    public double getThreshold() {
                        return mThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return mReadings.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return getMinReadings();
                    }

                    @Override
                    public void estimatePreliminarSolutions(int[] sampleIndices,
                            List<Solution<Point3D>> solutions) {
                        solvePreliminarSolutions(sampleIndices, solutions);
                    }

                    @Override
                    public double computeResidual(Solution<Point3D> currentEstimation, int i) {
                        return residual(currentEstimation, i);
                    }

                    @Override
                    public boolean isReady() {
                        return MSACRobustRangingRadioSourceEstimator3D.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(RobustEstimator<Solution<Point3D>> estimator) {
                        if (mListener != null) {
                            mListener.onEstimateStart(
                                    MSACRobustRangingRadioSourceEstimator3D.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(RobustEstimator<Solution<Point3D>> estimator) {
                        if (mListener != null) {
                            mListener.onEstimateEnd(
                                    MSACRobustRangingRadioSourceEstimator3D.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(RobustEstimator<Solution<Point3D>> estimator, int iteration) {
                        if (mListener != null) {
                            mListener.onEstimateNextIteration(
                                    MSACRobustRangingRadioSourceEstimator3D.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(RobustEstimator<Solution<Point3D>> robustEstimator,
                            float progress) {
                        if (mListener != null) {
                            mListener.onEstimateProgressChange(
                                    MSACRobustRangingRadioSourceEstimator3D.this,
                                    progress);
                        }
                    }
                });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Solution<Point3D> result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            attemptRefine(result);

        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }
}
