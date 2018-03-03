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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class to robustly estimate position and transmitted power of a WiFi access point, by discarding
 * outliers and assuming that the access point emits isotropically following the expression below:
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
 * @param <P> a {@link Point} type.
 */
public abstract class RobustWifiAccessPointPowerAndPositionEstimator<P extends Point> {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that result is refined by default using all found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
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
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Initial transmitted power to start the estimation of access point
     * transmitted power.
     * If not defined, average value of received power readings will be used.
     */
    protected Double mInitialTransmittedPowerdBm;

    /**
     * Initial position to start the estimation of access point position.
     * If not defined, centroid of provided fingerprints will be used.
     */
    protected P mInitialPosition;

    /**
     * WiFi signal readings belonging to the same access point to be estimated.
     */
    protected List<? extends WifiReadingLocated<P>> mReadings;

    /**
     * Listener to be notified of events such as when estimation starts, ends or its
     * progress significantly changes.
     */
    protected RobustWifiAccessPointPowerAndPositionEstimatorListener<P> mListener;

    /**
     * Estimated position.
     */
    protected P mEstimatedPosition;

    /**
     * Estimated transmitted power expressed in dBm's.
     */
    protected double mEstimatedTransmittedPowerdBm;

    /**
     * Indicates if this instance is locked because estimation is being executed.
     */
    protected boolean mLocked;

    /**
     * Amount of progress variation before notifying a progress change during estimation.
     */
    protected float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%). The amount of confidence indicates the probability that the estimated
     * result is correct. Usually this value will be close to 1.0, but not exactly 1.0.
     */
    protected double mConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations. When the maximum number of iterations is
     * exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     */
    protected int mMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData mInliersData;

    /**
     * Indicates whether result must be refined using found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    protected boolean mRefineResult = DEFAULT_REFINE_RESULT;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    protected boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Covariance of estimated position and power.
     * This is only available when result has been refined and covariance is kept.
     */
    protected Matrix mCovariance;

    /**
     * Constructor.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator() { }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings)
            throws IllegalArgumentException {
        internalSetReadings(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets WiFi signal readings bleonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener)
            throws IllegalArgumentException {
        this(readings);
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            P initialPosition)
            throws IllegalArgumentException {
        internalSetReadings(readings);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(P initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener) {
        mListener = listener;
        mInitialPosition = initialPosition;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            P initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition);
        mListener = listener;
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            Double initialTransmittedPowerdBm) {
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        internalSetReadings(readings);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener) {
        mListener = listener;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener)
            throws IllegalArgumentException {
        this(readings, initialTransmittedPowerdBm);
        mListener = listener;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        internalSetReadings(readings);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     */
    public RobustWifiAccessPointPowerAndPositionEstimator(P initialPosition,
            Double initialTransmittedPowerdBm) {
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(P initialPosition,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener) {
        mListener = listener;
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
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
    public RobustWifiAccessPointPowerAndPositionEstimator(
            List<? extends WifiReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        mListener = listener;
    }

    /**
     * Gets initial transmitted power to start the estimation of access point
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     * @return initial transmitted power to start the estimation of access point
     * transmitted power.
     */
    public Double getInitialTransmittedPowerdBm() {
        return mInitialTransmittedPowerdBm;
    }

    /**
     * Sets initial transmitted power to start the estimation of access point
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted
     *                                   power.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialTransmittedPowerdBm(Double initialTransmittedPowerdBm)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Gets initial transmitted power to start the estimation of access point
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     * @return initial transmitted power to start the estimation of access point
     * transmitted power.
     */
    public Double getInitialTransmittedPower() {
        return mInitialTransmittedPowerdBm != null ?
                Utils.dBmToPower(mInitialTransmittedPowerdBm) : null;
    }

    /**
     * Sets initial transmitted power to start the estimation of access point
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     * @param initialTransmittedPower initial transmitted power to start the
     *                                estimation of access point transmitted power.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setInitialTransmittedPower(Double initialTransmittedPower)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (initialTransmittedPower != null) {
            if (initialTransmittedPower < 0.0) {
                throw new IllegalArgumentException();
            }
            mInitialTransmittedPowerdBm = Utils.powerTodBm(
                    initialTransmittedPower);
        } else {
            mInitialTransmittedPowerdBm = null;
        }
    }

    /**
     * Gets initial position to start the estimation of access point position.
     * If not defined, centroid of provided fingerprints will be used.
     * @return initial position to start the estimation of access point position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the estimation of access point position.
     * If not defined, centroid of provided fingerprints will be used.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Indicates whether estimator is locked during estimation.
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setProgressDelta(float progressDelta)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that
     * the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException if estimator is locked.
     */
    public void setConfidence(double confidence)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling solve(),
     * a RobustEstimatorException will be raised.
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setMaxIterations(int maxIterations)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    public boolean areValidReadings(
            List<? extends WifiReadingLocated<P>> readings) {

        return readings != null && readings.size() >= getMinReadings();
    }

    /**
     * Gets WiFi signal readings belonging to the same access point.
     * @return WiFi signal readings belonging to the same access point.
     */
    public List<? extends WifiReadingLocated<P>> getReadings() {
        return mReadings;
    }

    /**
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same
     *                 access point.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public void setReadings(List<? extends WifiReadingLocated<P>> readings)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetReadings(readings);
    }

    /**
     * Gets listener in charge of attending events raised by this instance.
     * @return listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimatorListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener in charge of attending events raised by this instance.
     * @param listener listener in charge of attending events raised by this
     *                 instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<P> listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        return areValidReadings(mReadings);
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     * @return quality scores corresponding to each sample.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each pair of positions and
     * distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    public void setQualityScores(double[] qualityScores)
            throws IllegalArgumentException, LockedException { }

    /**
     * Gets covariance for estimated position and power.
     * Top-left submatrix contains covariance of position, and last diagonal element
     * contains variance of estimated transmitted power.
     * This is only available when result has been refined and covariance is kept.
     * @return covariance for estimated position and power.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated position covariance.
     * This is only available when result has been refined and covariance is kept.
     * @return estimated position covariance.
     */
    public Matrix getEstimatedPositionCovariance() {
        if (mCovariance == null) {
            return null;
        }

        int d = getNumberOfDimensions() - 1;
        return mCovariance.getSubmatrix(
                0, 0, d, d);
    }

    /**
     * Gets estimated transmitted power variance.
     * This is only available when result has been refined and covariance is kept.
     * @return estimated transmitted power variance.
     */
    public double getEstimatedTransmittedPowerVariance() {
        if (mCovariance == null) {
            return 0.0;
        }

        int last = mCovariance.getRows() - 1;
        return mCovariance.getElementAt(last, last);
    }

    /**
     * Gets estimated transmitted power expressed in milli watts (mW).
     * @return estimated transmitted power expressed in milli watts.
     */
    public double getEstimatedTransmittedPower() {
        return Utils.dBmToPower(mEstimatedTransmittedPowerdBm);
    }

    /**
     * Gets estimated transmitted power expressed in dBm's.
     * @return estimated transmitted power expressed in dBm's.
     */
    public double getEstimatedTransmittedPowerdBm() {
        return mEstimatedTransmittedPowerdBm;
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mEstimatedPosition;
    }

    /**
     * Gets minimum required number of readings to estimate
     * power and position.
     * This is 3 readings for 2D, and 4 readings for 3D.
     * @return minimum required number of readings.
     */
    public abstract int getMinReadings();

    /**
     * Gets number of dimensions of position points.
     * @return number of dimensions of position points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Robustly estimates position and transmitted power for an access point.
     * @throws LockedException if instance is busy during estimation.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    public abstract void estimate() throws LockedException, NotReadyException,
            RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internally sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @throws IllegalArgumentException if readings are null, not enough readings
     * are available, or readings do not belong to the same access point.
     */
    protected void internalSetReadings(
            List<? extends WifiReadingLocated<P>> readings)
            throws IllegalArgumentException {
        if (!areValidReadings(readings)) {
            throw new IllegalArgumentException();
        }

        mReadings = readings;
    }

    /**
     * Solves preliminar solution for a subset of samples.
     * @param samplesIndices indices of subset samples.
     * @param solutions instance where solution will be stored.
     */
    protected abstract void solvePreliminarSolutions(int[] samplesIndices,
            List<Solution<P>> solutions);

    /**
     * Estimates residual for a solution obtained for a subset of samples.
     * @param currentEstimation solution obtained for a subset of samples.
     * @param i i-th fingerprint to obtain residual for.
     * @return difference between measured and expected RSSI value.
     */
    protected double residual(Solution<P> currentEstimation, int i) {
        //Model fitted internally is equal to:
        //Pr (dBm) = 10 * log(Pte * k / d^2) = 10*log(k) + 10*log(Pte) - 20*log(d)
        //where;
        //Pr is received, expressed in dBm
        //Pte is equivalent transmitted power, expressed in dBm
        //k is a constant equal to k = c^2 / (pi * f)^2, where c is speed of light
        //and d is equal to distance between fingerprint and estimated position
        WifiReadingLocated<P> reading = mReadings.get(i);
        double frequency = reading.getAccessPoint().getFrequency();

        //compute k as the constant part of the isotropic received power formula
        //so that: Pr = Pte*k/d^2
        double k = Math.pow(WifiAccessPointPowerAndPositionEstimator.SPEED_OF_LIGHT /
                (4.0 * Math.PI * frequency), 2.0);
        final double kdB = 10.0 * Math.log10(k);

        //get distance from estimated access point position and fingerprint position
        P fingerprintPosition = reading.getPosition();
        P accessPointPosition = currentEstimation.getEstimatedPosition();
        double sqrDistance = accessPointPosition.sqrDistanceTo(fingerprintPosition);

        int dims = getNumberOfDimensions();
        double transmittedPowerdBm = currentEstimation.
                getEstimatedTransmittedPowerdBm();

        //compute expected received power assuming isotropic transmission
        //and compare agains measured RSSI at fingerprint location
        double expectedRSSI = kdB + transmittedPowerdBm - 10.0 * Math.log10(sqrDistance);
        double rssi = reading.getRssi();

        return Math.abs(expectedRSSI - rssi);
    }

    /**
     * Contains a solution obtained during robust estimation for a subset of
     * samples.
     * @param <P> a {@link Point} type.
     */
    static class Solution<P extends Point> {
        /**
         * Estimated position for a subset of samples.
         */
        private P mEstimatedPosition;

        /**
         * Estimated transmitted power expressed in dBm's for a subset of samples.
         */
        private double mEstimatedTransmittedPowerdBm;

        /**
         * Constructor.
         * @param estimatedPosition estimated position for a subset of samples.
         * @param estimatedTransmittedPowerdBm estimated transmitted power expressed
         *                                     in dBm's for a subset of samples.
         */
        public Solution(P estimatedPosition, double estimatedTransmittedPowerdBm) {
            mEstimatedPosition = estimatedPosition;
            mEstimatedTransmittedPowerdBm = estimatedTransmittedPowerdBm;
        }

        /**
         * Gets estimated position for a subset of samples.
         * @return estimated position for a subset of samples.
         */
        public P getEstimatedPosition() {
            return mEstimatedPosition;
        }

        /**
         * Gets estimated transmitted power expressed in dBm's for a subset of
         * samples.
         * @return estimated transmitted power expressed in dBm's for a subset
         * of samples.
         */
        public double getEstimatedTransmittedPowerdBm() {
            return mEstimatedTransmittedPowerdBm;
        }
    }
}
