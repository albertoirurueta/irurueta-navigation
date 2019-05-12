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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly estimate 2D position, transmitted power and pathloss exponent of a radio source
 * (e.g. WiFi access point or bluetooth beacon), by discarding outliers using MSAC
 * algorithm and assuming that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If RssiReadings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * Implementations of this class should be able to detect and discard outliers in
 * order to find the best solution.
 *
 * IMPORTANT: When using this class estimation can be done using a
 * combination of radio source position, transmitted power and path loss
 * exponent. However enabling all three estimations usually achieves
 * inaccurate results. When using this class, estimation must be of at least
 * one parameter (position, transmitted power or path loss exponent) when
 * initial values are provided for the other two, and at most it should consist
 * of two parameters (either position and transmitted power, position and
 * path loss exponent or transmitted power and path loss exponent), providing an
 * initial value for the remaining parameter.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("Duplicates")
public class MSACRobustRssiRadioSourceEstimator2D<S extends RadioSource> extends
        RobustRssiRadioSourceEstimator2D<S> {

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
    public MSACRobustRssiRadioSourceEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings          signal readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation of radio
     *                          source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public MSACRobustRssiRadioSourceEstimator2D(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition   initial position to start the estimation of radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     */
    public MSACRobustRssiRadioSourceEstimator2D(Point2D initialPosition,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings          signal readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation of radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's)
     * @param listener                      listener in charge of attending events raised by this instance.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's)
     * @param listener                      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialPosition               initial position to start the estimation of access
     *                                      point position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     */
    public MSACRobustRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param listener                      in charge of attending events raised by this instance.
     */
    public MSACRobustRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param listener                      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param initialPathLossExponent       initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
    }

    /**
     * Constructor.
     *
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param initialPathLossExponent       initial path loss exponent. A typical value is 2.0.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
    }

    /**
     * Constructor.
     *
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param initialPathLossExponent       initial path loss exponent. A typical value is 2.0.
     * @param listener                      listener in charge of attending events raised by this instance.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                      signal readings belonging to the same radio source.
     * @param initialPosition               initial position to start the estimation of radio
     *                                      source position.
     * @param initialTransmittedPowerdBm    initial transmitted power to start the
     *                                      estimation of radio source transmitted power
     *                                      (expressed in dBm's).
     * @param initialPathLossExponent       initial path loss exponent. A typical value is 2.0.
     * @param listener                      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MSACRobustRssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     * zero.
     * @throws LockedException if robust estimator is locked because an
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Robustly estimates position, transmitted power and pathloss exponent for a
     * radio source.
     *
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
                                return Math.max(mPreliminarySubsetSize, getMinReadings());
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
                                return MSACRobustRssiRadioSourceEstimator2D.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(RobustEstimator<Solution<Point2D>> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            MSACRobustRssiRadioSourceEstimator2D.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(RobustEstimator<Solution<Point2D>> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            MSACRobustRssiRadioSourceEstimator2D.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(RobustEstimator<Solution<Point2D>> estimator, int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            MSACRobustRssiRadioSourceEstimator2D.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(RobustEstimator<Solution<Point2D>> estimator, float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            MSACRobustRssiRadioSourceEstimator2D.this, progress);
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
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }
}
