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
package com.irurueta.navigation.fingerprinting;

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly estimate 2D position and transmitted power of a WiFi access point, by
 * discarding outliers using MSAC algorithm and assuming that the access point
 * emits isotropically following the expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antena
 * Gr is the Gain of the receiver antena
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the WiFi signal.
 * Because usually information about the antena of the Wifi Access Point cannot be
 * retrieved (because many measurements are made on unkown access points where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If WifiReadings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 * Implementations of this class should be able to detect and discard outliers in
 * order to find the best solution.
 */
public class MSACRobustWifiAccessPointPowerAndPositionEstimator2D extends
        RobustWifiAccessPointPowerAndPositionEstimator2D {

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
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Point2D initialPosition)
            throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(Point2D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Point2D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener in charge of attending events raised by this instance.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustWifiAccessPointPowerAndPositionEstimator2D(
            List<? extends WifiReadingLocated<Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
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
     * Robustly estimates position and transmitted power for an access point.
     * @throws LockedException if instance is busy during estimation.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public void estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        MSACRobustEstimator<Solution<Point2D>> innerEstimator =
                new MSACRobustEstimator<>(
                        new MSACRobustEstimatorListener<Solution<Point2D>>() {
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
                            public void estimatePreliminarSolutions(int[] samplesIndices,
                                                                    List<Solution<Point2D>> solutions) {
                                solvePreliminarSolutions(samplesIndices, solutions);
                            }

                            @Override
                            public double computeResidual(Solution<Point2D> currentEstimation, int i) {
                                return residual(currentEstimation, i);
                            }

                            @Override
                            public boolean isReady() {
                                return MSACRobustWifiAccessPointPowerAndPositionEstimator2D.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(RobustEstimator<Solution<Point2D>> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            MSACRobustWifiAccessPointPowerAndPositionEstimator2D.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(RobustEstimator<Solution<Point2D>> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            MSACRobustWifiAccessPointPowerAndPositionEstimator2D.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(RobustEstimator<Solution<Point2D>> estimator, int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            MSACRobustWifiAccessPointPowerAndPositionEstimator2D.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(RobustEstimator<Solution<Point2D>> estimator, float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            MSACRobustWifiAccessPointPowerAndPositionEstimator2D.this, progress);
                                }
                            }
                        });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Solution<Point2D> result = innerEstimator.estimate();
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
