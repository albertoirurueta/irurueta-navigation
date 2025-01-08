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
package com.irurueta.navigation.lateration;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

/**
 * This is an abstract class to robustly solve the lateration problem by finding the best
 * pairs of positions and distances among the provided ones.
 * Implementations of this class should be able to detect and discard outliers in order to find
 * the best solution.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class RobustLaterationSolver<P extends Point<?>> {

    /**
     * Indicates that by default a linear solver is used for preliminary solution estimation.
     * The result obtained on each preliminary solution might be later refined.
     */
    private static final boolean DEFAULT_USE_LINEAR_SOLVER = true;

    /**
     * Indicates that by default an homogeneous linear solver is used either to estimate preliminary
     * solutions or an initial solution for preliminary solutions that will be later refined.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER = false;

    /**
     * Indicates that by default preliminary solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_PRELIMINARY_SOLUTIONS = true;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROMEDS;

    /**
     * Indicates that result is refined by default using a non-linear lateration solver
     * (which uses Levenberg-Marquardt fitter).
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
     * By default, this is set to 5%.
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
     * accurate because chosen sub-samples will be inliers.
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
     * Minimum allowed distance for a given circle or sphere.
     */
    public static final double EPSILON = 1e-7;

    /**
     * Known positions of static nodes.
     */
    protected P[] positions;

    /**
     * Euclidean distances from static nodes to mobile node.
     */
    protected double[] distances;

    /**
     * Listener to be notified of events such as when solving starts, ends or its
     * progress significantly changes.
     */
    protected RobustLaterationSolverListener<P> listener;

    /**
     * Initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     */
    protected P initialPosition;

    /**
     * Indicates whether a linear solver is used or not (either homogeneous or inhomogeneous)
     * for preliminary solutions.
     */
    protected boolean useLinearSolver = DEFAULT_USE_LINEAR_SOLVER;

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate preliminary solutions
     * or an initial solution for preliminary solutions that will be later refined.
     */
    protected boolean useHomogeneousLinearSolver = DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution is found.
     */
    protected boolean refinePreliminarySolutions = DEFAULT_REFINE_PRELIMINARY_SOLUTIONS;

    /**
     * Estimated position.
     */
    protected P estimatedPosition;

    /**
     * Indicates if this instance is locked because lateration is being solved.
     */
    protected boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during estimation.
     */
    protected float progressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%). The amount of confidence indicates the probability that the estimated
     * result is correct. Usually this value will be close to 1.0, but not exactly 1.0.
     */
    protected double confidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations. When the maximum number of iterations is
     * exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     */
    protected int maxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData inliersData;

    /**
     * Indicates whether result must be refined using a non-linear lateration solver over
     * found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    protected boolean refineResult = DEFAULT_REFINE_RESULT;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    protected boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Estimated covariance of estimated position.
     * This is only available when result has been refined and covariance is kept.
     */
    protected Matrix covariance;

    /**
     * Standard deviations of provided distances.
     */
    protected double[] distanceStandardDeviations;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int preliminarySubsetSize;

    /**
     * Constructor.
     */
    protected RobustLaterationSolver() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustLaterationSolver(final RobustLaterationSolverListener<P> listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    protected RobustLaterationSolver(final P[] positions, final double[] distances) {
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length or their length is smaller than required
     *                                  (3 for 2D points or 4 for 3D points).
     */
    protected RobustLaterationSolver(
            final P[] positions, final double[] distances, final double[] distanceStandardDeviations) {
        internalSetPositionsDistancesAndStandardDeviations(positions, distances, distanceStandardDeviations);
    }


    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events such as when estimation starts, ends or its progress
     *                  significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    protected RobustLaterationSolver(
            final P[] positions, final double[] distances, final RobustLaterationSolverListener<P> listener) {
        internalSetPositionsAndDistances(positions, distances);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    protected RobustLaterationSolver(
            final P[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<P> listener) {
        internalSetPositionsDistancesAndStandardDeviations(positions, distances, distanceStandardDeviations);
        this.listener = listener;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public RobustLaterationSolverListener<P> getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setListener(final RobustLaterationSolverListener<P> listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Gets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @return an initial position.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @param initialPosition an initial position.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.initialPosition = initialPosition;
    }

    /**
     * Indicates whether a linear solver is used or not (either homogeneous or inhomogeneous)
     * for preliminary solutions.
     *
     * @return true if a linear solver is used, false otherwise.
     */
    public boolean isLinearSolverUsed() {
        return useLinearSolver;
    }

    /**
     * Specifies whether a linear solver is used or not (either homogeneous or inhomogeneous)
     * for preliminary solutions.
     *
     * @param linearSolverUsed true if a linear solver is used, false otherwise.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setLinearSolverUsed(final boolean linearSolverUsed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        useLinearSolver = linearSolverUsed;
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate preliminary solutions
     * or an initial solution for preliminary solutions that will be later refined.
     *
     * @return true if homogeneous linear solver is used, false otherwise.
     */
    public boolean isHomogeneousLinearSolverUsed() {
        return useHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate preliminary solutions
     * or an initial solution for preliminary solutions that will be later refined.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   otherwise.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setHomogeneousLinearSolverUsed(final boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.useHomogeneousLinearSolver = useHomogeneousLinearSolver;
    }

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non-linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @return true if preliminary solutions must be refined after an initial linear solution, false
     * otherwise.
     */
    public boolean isPreliminarySolutionRefined() {
        return refinePreliminarySolutions;
    }

    /**
     * Specifies whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non-linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @param preliminarySolutionRefined true if preliminary solutions must be refined after an
     *                                   initial linear solution, false otherwise.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setPreliminarySolutionRefined(final boolean preliminarySolutionRefined) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        refinePreliminarySolutions = preliminarySolutionRefined;
    }

    /**
     * Returns boolean indicating if solver is locked because estimation is under
     * progress.
     *
     * @return true if solver is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if this solver is locked because an estimation is being computed.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
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
        return confidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that
     * the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if solver is locked because an estimation is being computed.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling solve(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this solver is locked because an estimation is being
     *                                  computed.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.maxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return inliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return refineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if solver is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Gets known positions of static nodes.
     *
     * @return known positions of static nodes.
     */
    public P[] getPositions() {
        return positions;
    }

    /**
     * Gets euclidean distances from static nodes to mobile node.
     *
     * @return euclidean distances from static nodes to mobile node.
     */
    public double[] getDistances() {
        return distances;
    }

    /**
     * Gets standard deviations of provided distances.
     * This is used during refinement.
     *
     * @return standard deviations of provided distances.
     */
    public double[] getDistanceStandardDeviations() {
        return distanceStandardDeviations;
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    public boolean isReady() {
        return positions != null && distances != null && positions.length >= getMinRequiredPositionsAndDistances();
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
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if robust solver is locked because an
     *                                  estimation is already in progress.
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Gets estimated covariance of estimated position if available.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Sets known positions and Euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setPositionsAndDistances(final P[] positions, final double[] distances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Sets known positions, Euclidean distances and the respective standard deviations of
     * measured distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required
     *                                 (2 points).
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setPositionsDistancesAndStandardDeviations(
            final P[] positions, final double[] distances, final double[] distanceStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPositionsDistancesAndStandardDeviations(positions, distances, distanceStandardDeviations);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return estimatedPosition;
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return number of dimensions of provided points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Minimum required number of positions and distances.
     * This value will depend on actual implementation and whether we are solving a 2D or 3D problem.
     *
     * @return minimum required number of positions and distances.
     */
    public abstract int getMinRequiredPositionsAndDistances();

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinRequiredPositionsAndDistances()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return preliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinRequiredPositionsAndDistances()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if instance is busy solving the lateration problem.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinRequiredPositionsAndDistances()}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        this.preliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Solves the lateration problem.
     *
     * @return estimated position.
     * @throws LockedException          if instance is busy solving the lateration problem.
     * @throws NotReadyException        if solver is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract P solve() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internally sets known positions and Euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    protected void internalSetPositionsAndDistances(final P[] positions, final double[] distances) {
        if (positions == null || distances == null) {
            throw new IllegalArgumentException();
        }

        if (positions.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (positions.length != distances.length) {
            throw new IllegalArgumentException();
        }

        this.positions = positions;
        this.distances = distances;

        // fix distances if needed
        for (var i = 0; i < this.distances.length; i++) {
            if (this.distances[i] < EPSILON) {
                this.distances[i] = EPSILON;
            }
        }
    }

    /**
     * Internally sets known positions, Euclidean distances and the respective standard
     * deviations of measured distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    protected void internalSetPositionsDistancesAndStandardDeviations(
            final P[] positions, final double[] distances, final double[] distanceStandardDeviations) {
        if (distanceStandardDeviations == null || distances == null) {
            throw new IllegalArgumentException();
        }
        if (distances.length != distanceStandardDeviations.length) {
            throw new IllegalArgumentException();
        }
        internalSetPositionsAndDistances(positions, distances);
        this.distanceStandardDeviations = distanceStandardDeviations;
    }
}
