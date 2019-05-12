/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.lateration.NonLinearLeastSquaresLaterationSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolverListener;
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
 *
 * @param <P> a {@link Point} type.
 * @param <R> a {@link Reading} type.
 * @param <L> a {@link RobustPositionEstimatorListener} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class RobustPositionEstimator<P extends Point,
        R extends Reading<? extends RadioSource>,
        L extends RobustPositionEstimatorListener<? extends RobustPositionEstimator>> {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that by default located radio source position covariance is taken
     * into account (if available) to determine distance standard deviation.
     */
    public static final boolean DEFAULT_USE_RADIO_SOURCE_POSITION_COVARIANCE = true;

    /**
     * Indicates that by default readings are distributed evenly among radio sources
     * taking into account quality scores of both radio sources and readings.
     */
    public static final boolean DEFAULT_EVENLY_DISTRIBUTE_READINGS = true;

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            NonLinearLeastSquaresLaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;

    /**
     * Located radio sources  used for lateration.
     */
    protected List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Fingerprint containing readings at an unknown location for provided located
     * radio sources.
     */
    protected Fingerprint<? extends RadioSource, ? extends R> mFingerprint;

    /**
     * Indicates whether located radio source position covariances must be taken into
     * account (if available) to determine distance standard deviation.
     */
    private boolean mUseRadioSourcePositionCovariance =
            DEFAULT_USE_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Indicates whether readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and readings.
     */
    private boolean mEvenlyDistributeReadings = DEFAULT_EVENLY_DISTRIBUTE_READINGS;

    /**
     * Distance standard deviation fallback value to use when none can be determined
     * from provided radio sources and fingerprint readings.
     */
    private double mFallbackDistanceStandardDeviation =
            FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected L mListener;

    /**
     * A robust lateration solver to solve position.
     */
    protected RobustLaterationSolver<P> mLaterationSolver;

    /**
     * Listener for the robust lateration solver.
     */
    protected RobustLaterationSolverListener<P> mTrilaterationSolverListener;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int mPreliminarySubsetSize;

    /**
     * Constructor.
     */
    public RobustPositionEstimator() { }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustPositionEstimator(L listener) {
        mListener = listener;
    }

    /**
     * Gets located radio sources used for lateration.
     *
     * @return located radio sources used for lateration.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public void setSources(List<? extends RadioSourceLocated<P>> sources)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located
     * radio sources.
     *
     * @return  fingerprint containing readings at an unknown location for provided
     * located radio sources.
     */
    public Fingerprint<? extends RadioSource, ? extends R> getFingerprint() {
        return mFingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided located
     * radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprint(
            Fingerprint<? extends RadioSource, ? extends R> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(L listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Indicates whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     *
     * @return true to take radio source position covariance into account, false
     * otherwise.
     */
    public boolean isRadioSourcePositionCovarianceUsed() {
        return mUseRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     *
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
     * Indicates whether readings are evenly distributed among radio sources taking
     * into account quality scores of both radio sources and readings.
     *
     * @return true if readings are evenly distributed, false otherwise.
     */
    public boolean getEvenlyDistributeReadings() {
        return mEvenlyDistributeReadings;
    }

    /**
     * Specifies whether readings are evenly distributed among radio sources taking
     * into account quality scores of both radio sources and readings.
     *
     * @param evenlyDistributeReadings true if readings are evenly distributed, false
     *                                 otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setEvenlyDistributeReadings(boolean evenlyDistributeReadings)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mEvenlyDistributeReadings = evenlyDistributeReadings;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     *
     * @return distance standard deviation to use as fallback.
     */
    public double getFallbackDistanceStandardDeviation() {
        return mFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     *
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
     * Returns boolean indicating if estimator is locked because estimation is
     * under progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLaterationSolver.isLocked();
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mLaterationSolver.getProgressDelta();
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if progress delta is less than zero or greater
     *                                  than 1.
     */
    public void setProgressDelta(float progressDelta) throws LockedException {
        mLaterationSolver.setProgressDelta(progressDelta);
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that the
     * estimated result is correct. Usually this value will be close to 1.0, but not
     * exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mLaterationSolver.getConfidence();
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that the
     * estimated result is correct. Usually this value will be close to 1.0, but not
     * exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     */
    public void setConfidence(double confidence) throws LockedException {
        mLaterationSolver.setConfidence(confidence);
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling solve(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mLaterationSolver.getMaxIterations();
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is less than 1.
     */
    public void setMaxIterations(int maxIterations) throws LockedException {
        mLaterationSolver.setMaxIterations(maxIterations);
    }

    /**
     * Indicates whether result must be refined using a non-linear estimator over found
     * inliers.
     *
     * @return true to refine result, false to simply use result found by robust
     * estimator without further refining.
     */
    public boolean isResultRefined() {
        return mLaterationSolver.isResultRefined();
    }

    /**
     * Specifies whether result must be refined using a non-linear estimator over found
     * inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by
     *                     robust estimator without further refining.
     * @throws LockedException if this instance is locked.
     */
    public void setResultRefined(boolean refineResult) throws LockedException {
        mLaterationSolver.setResultRefined(refineResult);
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mLaterationSolver.isCovarianceKept();
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setCovarianceKept(boolean keepCovariance) throws LockedException {
        mLaterationSolver.setCovarianceKept(keepCovariance);
    }

    /**
     * Gets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @return an initial position.
     */
    public P getInitialPosition() {
        return mLaterationSolver.getInitialPosition();
    }

    /**
     * Sets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @param initialPosition an initial position.
     * @throws LockedException if this instance is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        mLaterationSolver.setInitialPosition(initialPosition);
    }

    /**
     * Indicates whether a linear solver is used or not (either homogeneous or
     * inhomogeneous) for preliminary solutions.
     *
     * @return true if a linear solver is used, false otherwise.
     */
    public boolean isLinearSolverUsed() {
        return mLaterationSolver.isLinearSolverUsed();
    }

    /**
     * Specifies whether a linear solver is used or not (either homogeneous or
     * inhomogeneous) for preliminary solutions.
     *
     * @param linearSolverUsed true if a linear solver is used, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setLinearSolverUsed(boolean linearSolverUsed) throws LockedException {
        mLaterationSolver.setLinearSolverUsed(linearSolverUsed);
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that will
     * be later refined.
     *
     * @return true if homogeneous linear solver is used, false otherwise.
     */
    public boolean isHomogeneousLinearSolverUsed() {
        return mLaterationSolver.isHomogeneousLinearSolverUsed();
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that will
     * be later refined.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used,
     *                                   false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setHomogeneousLinearSolverUsed(boolean useHomogeneousLinearSolver)
            throws LockedException {
        mLaterationSolver.setHomogeneousLinearSolverUsed(useHomogeneousLinearSolver);
    }

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear
     * solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will
     * be used regardless of this value using an average solution as the initial value
     * to be refined.
     *
     * @return true if preliminary solutions must be refined after an initial linear
     * solution, false otherwise.
     */
    public boolean isPreliminarySolutionRefined() {
        return mLaterationSolver.isPreliminarySolutionRefined();
    }

    /**
     * Specifies whether preliminary solutions must be refined after an initial linear
     * solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will
     * be used regardless of this value using an average solution as the initial value
     * to be refined.
     *
     * @param preliminarySolutionRefined true if preliminary solutions must be refined
     *                                   after an initial linear solution, false
     *                                   otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPreliminarySolutionRefined(boolean preliminarySolutionRefined)
            throws LockedException {
        mLaterationSolver.setPreliminarySolutionRefined(preliminarySolutionRefined);
    }

    /**
     * Gets data related to inliers found after estimation.
     * Inlier data is related to the internal positions and distances used for
     * solving lateration.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mLaterationSolver.getInliersData();
    }

    /**
     * Gets known positions of radio sources used internally to solve lateration.
     *
     * @return known positions used internally.
     */
    public P[] getPositions() {
        return mLaterationSolver.getPositions();
    }

    /**
     * Gets euclidean distances from known located radio sources to the location of
     * provided readings in a fingerprint.
     * Distance values are used internally to solve lateration.
     *
     * @return euclidean distances used internally.
     */
    public double[] getDistances() {
        return mLaterationSolver.getDistances();
    }

    /**
     * Gets standard deviation distances from known located radio sources to the
     * location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve lateration.
     *
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return mLaterationSolver.getDistanceStandardDeviations();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mLaterationSolver.isReady();
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each radio source.
     */
    public double[] getSourceQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the radio source.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @param sourceQualityScores quality scores corresponding to each radio source.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setSourceQualityScores(double[] sourceQualityScores)
            throws LockedException { }

    /**
     * Gets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each reading within provided
     * fingerprint.
     */
    public double[] getFingerprintReadingsQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @param fingerprintReadingsQualityScores quality scores corresponding to each
     *                                         reading within provided fingerprint.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setFingerprintReadingsQualityScores(
            double[] fingerprintReadingsQualityScores) throws LockedException { }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinRequiredSources()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinRequiredSources()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException if instance is busy solving the lateration problem.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinRequiredSources()}.
     */
    public void setPreliminarySubsetSize(int preliminarySubsetSize) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Gets estimated covariance of estimated position if available.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return mLaterationSolver.getCovariance();
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mLaterationSolver.getEstimatedPosition();
    }

    /**
     * Gets number of dimesnions of provided points.
     *
     * @return number of dimensions of provided points.
     */
    public int getNumberOfDimensions() {
        return mLaterationSolver.getNumberOfDimensions();
    }

    /**
     * Estimates position based on provided located radio sources and readings of such
     * sources at an unknown location.
     *
     * @return estimated position.
     * @throws LockedException          if estimator is locked.
     * @throws NotReadyException        if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for some other reason.
     */
    public P estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        mLaterationSolver.setPreliminarySubsetSize(mPreliminarySubsetSize);
        return mLaterationSolver.solve();
    }

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform
     * lateration.
     */
    public abstract int getMinRequiredSources();

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internally sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     * provided sources is less than the required minimum.
     */
    @SuppressWarnings("Duplicates")
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources) {
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
     * Internally sets fingerprint containing readings at an unknown location for
     * provided located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends R> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        mFingerprint = fingerprint;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
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
     * Builds positions, distances, standard deviation of distances and quality scores
     * for the internal lateration solver.
     */
    @SuppressWarnings("Duplicates")
    protected void buildPositionsDistancesDistanceStandardDeviationsAndQualityScores() {
        if (mLaterationSolver == null) {
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

        double[] sourceQualityScores = getSourceQualityScores();
        double[] fingerprintReadingsQualityScores =
                getFingerprintReadingsQualityScores();

        if (mEvenlyDistributeReadings) {
            // distribute evenly by modifying the relative values of quality scores
            if (sourceQualityScores == null) {
                sourceQualityScores = new double[mSources.size()];
            }
            if (fingerprintReadingsQualityScores == null) {
                fingerprintReadingsQualityScores = new double[
                        mFingerprint.getReadings().size()];
            }

            ReadingSorter<P, R> sorter = new ReadingSorter<>(mSources, mFingerprint,
                    sourceQualityScores, fingerprintReadingsQualityScores);
            sorter.sort();

            List<ReadingSorter.RadioSourceSourceWithQualityScore<P, R>> sortedSources =
                    sorter.getSortedSourcesAndReadings();

            int j = 0;
            int k = 0;
            boolean finished;
            do {
                int i = 0;
                finished = true;
                for (ReadingSorter.RadioSourceSourceWithQualityScore<P, R> sortedSource : sortedSources) {
                    sourceQualityScores[sortedSource.position] = i;
                    i--;

                    List<ReadingSorter.ReadingWithQualityScore<R>> sortedReadings =
                            sortedSource.readingsWithQualityScores;
                    if (k < sortedReadings.size()) {
                        finished = false;
                        ReadingSorter.ReadingWithQualityScore<R> sortedReading = sortedReadings.get(k);

                        fingerprintReadingsQualityScores[sortedReading.position] = j;
                        j--;
                    }
                }
                k++;
            } while (!finished);
        }

        List<Double> distanceQualityScores = null;
        if (sourceQualityScores != null || fingerprintReadingsQualityScores != null) {
            distanceQualityScores = new ArrayList<>();
        }
        PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                mSources, mFingerprint, sourceQualityScores,
                fingerprintReadingsQualityScores,
                isRadioSourcePositionCovarianceUsed(),
                getFallbackDistanceStandardDeviation(), positions, distances,
                distanceStandardDeviations, distanceQualityScores);

        setPositionsDistancesDistanceStandardDeviationsAndQualityScores(positions, distances,
                distanceStandardDeviations, distanceQualityScores);
    }
}
