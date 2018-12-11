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
package com.irurueta.navigation.indoor.position;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolver;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolverListener;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for robust position estimators using located radio sources and their
 * readings at unknown locations.
 * These kind of estimators can be used to robustly determine the position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * locations are known.
 * Implementations of this class should be able to detect and discard outliers in order
 * to find the best solution.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustPositionEstimator<P extends Point> {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Located radio sources used for trilateration.
     */
    protected List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Fingerprint containing readings at an unknown location for provided location radio sources.
     */
    protected Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> mFingerprint;

    /**
     * Indicates whether located radio source position covariance must be taken
     * into account (if available) to determine distance standard deviation.
     */
    private boolean mUseRadioSourcePositionCovariance;

    /**
     * Distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     */
    private double mFallbackDistanceStandardDeviation =
            FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected RobustPositionEstimatorListener<P> mListener;

    /**
     * A robust trilateration solver to solve position.
     */
    protected RobustTrilaterationSolver<P> mTrilaterationSolver;

    /**
     * Listener for the robust trilateration solver.
     */
    protected RobustTrilaterationSolverListener<P> mTrilaterationSolverListener;

    /**
     * Constructor.
     */
    public RobustPositionEstimator() {
        init();
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public RobustPositionEstimator(RobustPositionEstimatorListener<P> listener) {
        mListener = listener;
        init();
    }

    /**
     * Gets located radio sources used for trilateration.
     * @return located radio sources used for trilateration.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    public void setSources(List<? extends RadioSourceLocated<P>> sources)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located radio sources.
     * @return fingerprint containing readings at an unknown location for provided located radio sources.
     */
    public Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> getFingerprint() {
        return mFingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     * @return listener to be notified of events raised by this instance.
     */
    public RobustPositionEstimatorListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(RobustPositionEstimatorListener<P> listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Indicates whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     * @return true to take radio source position covariance into account, false
     * otherwise.
     */
    public boolean isRadioSourcePositionCovarianceUsed() {
        return mUseRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     * @param useRadioSourcePositionCovariance true to take radio source position
     *                                         covariance into account, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovarianceUsed(
            boolean useRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRadioSourcePositionCovariance = useRadioSourcePositionCovariance;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     * @return distance standard deviation to use as fallback.
     */
    public double getFallbackDistanceStandardDeviation() {
        return mFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     * @param fallbackDistanceStandardDeviation distance standard deviation to use
     *                                          as fallback.
     * @throws LockedException if estimator is locked.
     */
    public void setFallbackDistanceStandardDeviation(
            double fallbackDistanceStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mFallbackDistanceStandardDeviation = fallbackDistanceStandardDeviation;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is under
     * progress.
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mTrilaterationSolver.isLocked();
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mTrilaterationSolver.getProgressDelta();
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException if this solver is locked because an estimation is being computed.
     */
    public void setProgressDelta(float progressDelta)
            throws IllegalArgumentException, LockedException {
        mTrilaterationSolver.setProgressDelta(progressDelta);
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mTrilaterationSolver.getConfidence();
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that
     * the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException if solver is locked because an estimation is being computed.
     */
    public void setConfidence(double confidence)
            throws IllegalArgumentException, LockedException {
        mTrilaterationSolver.setConfidence(confidence);
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling solve(),
     * a RobustEstimatorException will be raised.
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mTrilaterationSolver.getMaxIterations();
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked because an estimation is being
     * computed.
     */
    public void setMaxIterations(int maxIterations)
            throws IllegalArgumentException, LockedException {
        mTrilaterationSolver.setMaxIterations(maxIterations);
    }

    /**
     * Indicates whether result must be refined using a non-linear estimator over found
     * inliers.
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mTrilaterationSolver.isResultRefined();
    }

    /**
     * Specifies whether result must be refined using a non-linear estimator over found
     * inliers.
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if solver is locked.
     */
    public void setResultRefined(boolean refineResult) throws LockedException {
        mTrilaterationSolver.setResultRefined(refineResult);
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mTrilaterationSolver.isCovarianceKept();
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(boolean keepCovariance) throws LockedException {
        mTrilaterationSolver.setCovarianceKept(keepCovariance);
    }

    /**
     * Gets data related to inliers found after estimation.
     * Inlier data is related to the internal positions and distances used
     * for solving trilateration.
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mTrilaterationSolver.getInliersData();
    }

    /**
     * Gets known positions of radio sources used internally to solve trilateration.
     * @return known positions used internally.
     */
    public P[] getPositions() {
        return mTrilaterationSolver.getPositions();
    }

    /**
     * Gets euclidean distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance values are used internally to solve trilateration.
     * @return euclidean distances used internally.
     */
    public double[] getDistances() {
        return mTrilaterationSolver.getDistances();
    }

    /**
     * Gets standard deviation distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve trilateration.
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return mTrilaterationSolver.getDistanceStandardDeviations();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mTrilaterationSolver.isReady();
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     * @return quality scores corresponding to each sample.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each radio
     *                      source.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    public void setQualityScores(double[] qualityScores)
            throws IllegalArgumentException, LockedException { }

    /**
     * Gets estimated covariance of estimated position if available.
     * This is only available when result has been refined and covariance
     * is kept.
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return mTrilaterationSolver.getCovariance();
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mTrilaterationSolver.getEstimatedPosition();
    }

    /**
     * Gets number of dimensions of provided points.
     * @return number of dimensions of provided points.
     */
    public int getNumberOfDimensions() {
        return mTrilaterationSolver.getNumberOfDimensions();
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @return estimated position.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for some other reason.
     */
    public P estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        return mTrilaterationSolver.solve();
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     * @return minimum required number of located radio sources to perform trilateration.
     */
    public abstract int getMinRequiredSources();

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internally sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources)
            throws IllegalArgumentException {
        if (sources == null) {
            throw new IllegalArgumentException();
        }

        if (sources.size() < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mSources = sources;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        mFingerprint = fingerprint;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal trilateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     * @param distanceQualityScores distance quality scores or null if not required.
     */
    protected abstract void setPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            List<P> positions, List<Double> distances,
            List<Double> distanceStandardDeviations,
            List<Double> distanceQualityScores);

    /**
     * Initializes robust trilateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new RobustTrilaterationSolverListener<P>() {
            @Override
            public void onSolveStart(RobustTrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(RobustPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(RobustTrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(RobustPositionEstimator.this);
                }
            }

            @Override
            public void onSolveNextIteration(RobustTrilaterationSolver<P> solver,
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RobustPositionEstimator.this, iteration);
                }
            }

            @Override
            public void onSolveProgressChange(RobustTrilaterationSolver<P> solver,
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RobustPositionEstimator.this, progress);
                }
            }
        };
    }

    /**
     * Builds positions, distances, standard deviation of distances and quality scores
     * for the internal trilateration solver.
     */
    protected void buildPositionsDistancesDistanceStandardDeviationsAndQualityScores() {
        if (mTrilaterationSolver == null) {
            return;
        }

        int min = getMinRequiredSources();
        if (mSources == null || mFingerprint == null ||
                mSources.size() < min ||
                mFingerprint.getReadings() == null ||
                mFingerprint.getReadings().size() < min) {
            return;
        }

        List<P> positions = new ArrayList<>();
        List<Double> distances = new ArrayList<>();
        List<Double> distanceStandardDeviations = new ArrayList<>();

        double[] qualityScores = getQualityScores();
        List<Double> distanceQualityScores = null;
        if (qualityScores != null) {
            distanceQualityScores = new ArrayList<>();
        }
        PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                mSources, mFingerprint, qualityScores,
                mUseRadioSourcePositionCovariance,
                mFallbackDistanceStandardDeviation, positions, distances,
                distanceStandardDeviations, distanceQualityScores);

        setPositionsDistancesDistanceStandardDeviationsAndQualityScores(positions, distances,
                distanceStandardDeviations, distanceQualityScores);
    }
}
