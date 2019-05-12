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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.ReadingLocated;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates a radio source. Usually this implies at least the estimation of
 * the radio source location, however, implementations of this class might estimate
 * additional parameters.
 *
 * @param <P> a {@link Point} type.
 * @param <R> a {@link ReadingLocated} type.
 * @param <L> a {@link RobustRadioSourceEstimatorListener} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustRadioSourceEstimator<P extends Point,
        R extends ReadingLocated<P>,
        L extends RobustRadioSourceEstimatorListener<? extends RobustRadioSourceEstimator>> {

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
     * Signal readings belonging to the same radio source to be estimated.
     */
    protected List<? extends R> mReadings;

    /**
     * Listener to be notified of events such as when estimation starts, ends or its
     * progress significantly changes.
     */
    protected L mListener;

    /**
     * Estimated position.
     */
    protected P mEstimatedPosition;

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
     * Covariance of estimated position, power and/or pathloss exponent.
     * This is only available when result has been refined and covariance is kept.
     */
    protected Matrix mCovariance;

    /**
     * Covariance of estimated position.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This value will only be available when position estimation is enabled.
     */
    protected Matrix mEstimatedPositionCovariance;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int mPreliminarySubsetSize;

    /**
     * Constructor.
     */
    public RobustRadioSourceEstimator() { }

    /**
     * Constructor.
     * Sets located radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRadioSourceEstimator(List<? extends R> readings) {
        internalSetReadings(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRadioSourceEstimator(L listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets located radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRadioSourceEstimator(List<? extends R> readings, L listener) {
        this(readings);
        mListener = listener;
    }

    /**
     * Indicates whether estimator is locked during estimation.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setProgressDelta(float progressDelta) throws LockedException {
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
     *
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
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException if estimator is locked.
     */
    public void setConfidence(double confidence) throws LockedException {
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
     * iterations is achieved without converging to a result when calling estimate(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setMaxIterations(int maxIterations) throws LockedException {
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
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
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
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
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
     * Gets signal readings belonging to the same radio source.
     *
     * @return signal readings belonging to the same radio source.
     */
    public List<? extends R> getReadings() {
        return mReadings;
    }

    /**
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same
     *                 radio source.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public void setReadings(List<? extends R> readings)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetReadings(readings);
    }

    /**
     * Gets listener in charge of attending events raised by this instance.
     *
     * @return listener in charge of attending events raised by this instance.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener in charge of attending events raised by this instance.
     *
     * @param listener listener in charge of attending events raised by this
     *                 instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(L listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
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
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    public void setQualityScores(double[] qualityScores)
            throws LockedException { }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinReadings()}
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during estimation.
     * This has to be at least {@link #getMinReadings()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException if instance is busy solving the lateration problem.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinReadings()}.
     */
    public void setPreliminarySubsetSize(int preliminarySubsetSize) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Gets covariance for estimated position, power and pathloss.
     * Matrix contains information in the following order:
     * Top-left submatrix contains covariance of position,
     * then follows transmitted power variance, and finally
     * the last element contains pathloss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return covariance for estimated position and power.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated position covariance.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated position covariance.
     */
    public Matrix getEstimatedPositionCovariance() {
        return mEstimatedPositionCovariance;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mEstimatedPosition;
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     *
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    public boolean areValidReadings(List<? extends R> readings) {
        return readings != null && readings.size() >= getMinReadings();
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Gets minimum required number of readings to estimate
     * power, position and pathloss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings for 2D, and 4 readings for 3D.
     *
     * @return minimum required number of readings.
     */
    public abstract int getMinReadings();

    /**
     * Gets number of dimensions of position points.
     *
     * @return number of dimensions of position points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Robustly estimates position, transmitted power and pathloss exponent for a
     * radio source.
     *
     * @throws LockedException if instance is busy during estimation.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    public abstract void estimate() throws LockedException, NotReadyException,
            RobustEstimatorException;

    /**
     * Gets estimated located radio source.
     *
     * @return estimated located radio source.
     * @param <S> type of located radio source.
     */
    public abstract <S extends RadioSourceLocated<P>> S getEstimatedRadioSource();

    /**
     * Internally sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are null, not enough readings
     * are available, or readings do not belong to the same access point.
     */
    protected void internalSetReadings(List<? extends R> readings) {
        if (!areValidReadings(readings)) {
            throw new IllegalArgumentException();
        }

        mReadings = readings;
    }
}
