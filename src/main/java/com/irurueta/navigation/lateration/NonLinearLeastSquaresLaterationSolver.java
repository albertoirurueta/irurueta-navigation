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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;

import java.util.Arrays;

/**
 * Solves a Trilateration problem with an instance of a least squares optimizer.
 * By solving the lateration problem linearly, this class is able to estimate
 * the covariance of estimated position.
 * To achieve better results, it is usually better to provide an initial coarse
 * solution.
 * This class is base on the implementation found at: https://github.com/lemmingapex/trilateration
 *
 * @param <P> a {@link Point} type.
 */
public abstract class NonLinearLeastSquaresLaterationSolver<P extends Point<?>> extends LaterationSolver<P> {

    /**
     * Default standard deviation assumed for provided distances when none is
     * explicitly provided.
     */
    public static final double DEFAULT_DISTANCE_STANDARD_DEVIATION = 1.0e-3;

    /**
     * Levenberg-Marquardt  fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter mFitter =
            new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Estimated covariance matrix for estimated position.
     */
    private Matrix mCovariance;

    /**
     * Estimated chi square value.
     */
    private double mChiSq;

    /**
     * Initial position to start lateration solving.
     * If not defined, centroid of provided position points will be used.
     */
    private P mInitialPosition;

    /**
     * Standard deviations of provided distances.
     */
    private double[] mDistanceStandardDeviations;

    /**
     * Constructor.
     */
    public NonLinearLeastSquaresLaterationSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required 2 points.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances) {
        super();
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     */
    public NonLinearLeastSquaresLaterationSolver(final P initialPosition) {
        super();
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param positions       known positions of static nodes.
     * @param distances       euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start lateration solving.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances, final P initialPosition) {
        this(initialPosition);
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final LaterationSolverListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required 2 points.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances,
            final LaterationSolverListener<P> listener) {
        super(listener);
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P initialPosition, final LaterationSolverListener<P> listener) {
        super(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param positions       known positions of static nodes.
     * @param distances       euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances, final P initialPosition,
            final LaterationSolverListener<P> listener) {
        this(initialPosition, listener);
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        super();
        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final P initialPosition) {
        this(initialPosition);
        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final LaterationSolverListener<P> listener) {
        super(listener);
        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final P initialPosition,
            final LaterationSolverListener<P> listener) {
        this(initialPosition, listener);
        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Sets known positions and euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    @Override
    public void setPositionsAndDistances(final P[] positions, final double[] distances)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Sets known positions, euclidean distances and the respective standard deviations of
     * measured distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required
     *                                  (2 points).
     * @throws LockedException          if instance is busy solving the trilateration problem.
     */
    public void setPositionsDistancesAndStandardDeviations(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Gets standard deviations of provided distances.
     *
     * @return standard deviations of provided distances.
     */
    public double[] getDistanceStandardDeviations() {
        return mDistanceStandardDeviations;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     *
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return mChiSq;
    }

    /**
     * Gets initial position to start lateration solving.
     * If not defined, centroid of provided position points will be used to start lateration.
     *
     * @return initial position to start lateration.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start lateration solving.
     * If not defined, centroid of provided position points will be used to start lateration.
     *
     * @param initialPosition initial position to start lateration.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Solves the lateration problem.
     *
     * @throws LaterationException if lateration fails.
     * @throws NotReadyException   is solver is not ready.
     * @throws LockedException     if instance is busy solving the lateration problem.
     */
    @Override
    public void solve() throws LaterationException, NotReadyException,
            LockedException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onSolveStart(this);
            }

            setupFitter();

            mFitter.fit();

            // estimated position
            mEstimatedPositionCoordinates = mFitter.getA();
            mCovariance = mFitter.getCovar();
            mChiSq = mFitter.getChisq();

            if (mListener != null) {
                mListener.onSolveEnd(this);
            }
        } catch (final NumericalException e) {
            throw new LaterationException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets lateration solver type.
     *
     * @return lateration solver type.
     */
    @Override
    public LaterationSolverType getType() {
        return LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER;
    }

    /**
     * Internally sets known positions and euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    @Override
    protected void internalSetPositionsAndDistances(
            final P[] positions, final double[] distances) {
        super.internalSetPositionsAndDistances(positions, distances);

        // initialize distances standard deviations to default values
        mDistanceStandardDeviations = new double[distances.length];
        Arrays.fill(mDistanceStandardDeviations, DEFAULT_DISTANCE_STANDARD_DEVIATION);
    }

    /**
     * Internally sets known positions, euclidean distances and the respective standard deviations of
     * measured distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    protected void internalSetPositionsDistancesAndStandardDeviations(
            final P[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        if (distanceStandardDeviations == null || distances == null) {
            throw new IllegalArgumentException();
        }
        if (distances.length != distanceStandardDeviations.length) {
            throw new IllegalArgumentException();
        }

        super.internalSetPositionsAndDistances(positions, distances);
        mDistanceStandardDeviations = distanceStandardDeviations;
    }

    /**
     * Setups fitter to solve lateration problem.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitter() throws FittingException {
        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        return NonLinearLeastSquaresLaterationSolver.this.getNumberOfDimensions();
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        final int dims = getNumberOfDimensions();
                        final double[] initial = new double[dims];

                        if (mInitialPosition == null) {
                            // use centroid of positions as initial value
                            final int numSamples = mPositions.length;

                            for (int i = 0; i < dims; i++) {
                                initial[i] = 0.0;
                                for (final P position : mPositions) {
                                    initial[i] += position.getInhomogeneousCoordinate(i) / (double) numSamples;
                                }
                            }
                        } else {
                            // use provided initial position
                            for (int i = 0; i < dims; i++) {
                                initial[i] = mInitialPosition.getInhomogeneousCoordinate(i);
                            }
                        }

                        return initial;
                    }

                    @Override
                    public double evaluate(
                            final int i, final double[] point, final double[] params,
                            final double[] derivatives) {
                        // we want to estimate the position contained as inhomogeneous coordinates in params array.
                        // the function evaluates the distance to provided point respect to current parameter
                        // (estimated position)
                        // sqrDist = (x - px)^2 + (y - py)^2
                        // grad = [2*(x - px), 2*(y - py)]
                        final int dims = getNumberOfDimensions();
                        double result = 0.0;
                        double diff;
                        double param;
                        for (int j = 0; j < dims; j++) {
                            param = params[j];
                            diff = param - point[j];
                            result += diff * diff;
                            derivatives[j] = 2.0 * diff;
                        }

                        return result;
                    }
                });

        final int dims = getNumberOfDimensions();
        double dist;

        try {
            final Matrix x = new Matrix(mPositions.length, dims);
            final double[] y = new double[mPositions.length];
            for (int i = 0; i < mPositions.length; i++) {
                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, mPositions[i].getInhomogeneousCoordinate(j));
                }

                dist = mDistances[i];
                y[i] = dist * dist;
            }

            mFitter.setInputData(x, y, mDistanceStandardDeviations);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }
}
