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
package com.irurueta.navigation.trilateration;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.*;

import java.util.Arrays;

/**
 * Solves a Trilateration problem with an instance of a least squares optimizer.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class NonLinearLeastSquaresTrilaterationSolver<P extends Point> extends TrilaterationSolver<P> {

    /**
     * Default standard deviation assumed for provided distances when none is
     * explicitly provided.
     */
    public static final double DEFAULT_DISTANCE_STANDARD_DEVIATION = 1.0e-3;

    /**
     * Levenberg-Marquardt  fitter to find a non-linear solution.
     */
    private LevenbergMarquardtMultiDimensionFitter mFitter = new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Estimated covariance matrix for estimated position.
     */
    private Matrix mCovariance;

    /**
     * Estimated chi square value.
     */
    private double mChiSq;

    /**
     * Initial position to start trilateration solving.
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
    public NonLinearLeastSquaresTrilaterationSolver() {
        super();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances) throws IllegalArgumentException {
        super();
        setPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start trilateration solving.
     */
    public NonLinearLeastSquaresTrilaterationSolver(P initialPosition) {
        super();
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances, P initialPosition)
            throws IllegalArgumentException {
        this(initialPosition);
        setPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(TrilaterationSolverListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            TrilaterationSolverListener<P> listener) throws IllegalArgumentException {
        super(listener);
        setPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(P initialPosition, TrilaterationSolverListener<P> listener) {
        super(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    @SuppressWarnings("unchecked")
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances, P initialPosition,
            TrilaterationSolverListener<P> listener) throws IllegalArgumentException {
        this(initialPosition, listener);
        setPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super();
        setPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            double[] distanceStandardDeviations, P initialPosition)
            throws IllegalArgumentException {
        this(initialPosition);
        setPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            double[] distanceStandardDeviations, TrilaterationSolverListener<P> listener)
            throws IllegalArgumentException {
        super(listener);
        setPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            double[] distanceStandardDeviations, P initialPosition,
            TrilaterationSolverListener<P> listener) throws IllegalArgumentException {
        this(initialPosition, listener);
        setPositionsDistancesAndStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }

    /**
     * Sets known positions and euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     * @param positions known positios of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    @Override
    public void setPositionsAndDistances(P[] positions, double[] distances) throws IllegalArgumentException {
        super.setPositionsAndDistances(positions, distances);

        //initialize distances standard deviations to default values
        mDistanceStandardDeviations = new double[distances.length];
        Arrays.fill(mDistanceStandardDeviations, DEFAULT_DISTANCE_STANDARD_DEVIATION);
    }

    /**
     * Sets known positions, euclidean distances and the respective standard deviations of
     * measured distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     * @param positions known positios of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public void setPositionsDistancesAndStandardDeviations(P[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        if(distanceStandardDeviations == null || distances == null) {
            throw new IllegalArgumentException();
        }
        if(distances.length != distanceStandardDeviations.length) {
            throw new IllegalArgumentException();
        }

        super.setPositionsAndDistances(positions, distances);
        mDistanceStandardDeviations = distanceStandardDeviations;
    }

    /**
     * Gets standard deviations of provided distances.
     * @return standard deviations of provided distances.
     */
    public double[] getDistanceStandardDeviations() {
        return mDistanceStandardDeviations;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated chi square value.
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return mChiSq;
    }

    /**
     * Gets initial position to start trilateration solving.
     * If not defined, centroid of provided position points will be used to start trilateration.
     * @return initial position to start trilateration.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Solves the trilateration problem.
     * @throws TrilaterationException if trilateration fails.
     * @throws NotReadyException is solver is not ready.
     */
    @Override
    @SuppressWarnings("unchecked")
    public void solve() throws TrilaterationException, NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            if (mListener != null) {
                mListener.onSolveStart(this);
            }

            setupFitter();

            mFitter.fit();

            //estimated position
            mEstimatedPositionCoordinates = mFitter.getA();
            mCovariance = mFitter.getCovar();
            mChiSq = mFitter.getChisq();

            if (mListener != null) {
                mListener.onSolveEnd(this);
            }
        } catch (NumericalException e) {
            throw new TrilaterationException(e);
        }
    }

    /**
     * Setups fitter to solve trilateration problem.
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitter() throws FittingException{
        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return NonLinearLeastSquaresTrilaterationSolver.this.getNumberOfDimensions();
            }

            @Override
            public double[] createInitialParametersArray() {
                int dims = getNumberOfDimensions();
                double[] initial = new double[dims];

                if (mInitialPosition == null) {
                    //use centroid of positions as initial value
                    int numSamples = mPositions.length;

                    for (int i = 0; i < dims; i++) {
                        initial[i] = 0.0;
                        for (P position : mPositions) {
                            initial[i] += position.getInhomogeneousCoordinate(i) / (double) numSamples;
                        }
                    }
                } else {
                    //use provided initial position
                    for (int i = 0; i < dims; i++) {
                        initial[i] = mInitialPosition.getInhomogeneousCoordinate(i);
                    }
                }

                return initial;
            }

            @Override
            public double evaluate(int i, double[] point, double[] params, double[] derivatives) {
                //we want to estimate the position contained as inhomogeneous coordinates in params array.
                //the function evaluates the distance to provided point respect to current parameter
                // (estimated position)
                //sqrDist = (x - px)^2 + (y - py)^2
                //grad = [2*(x - px), 2*(y - py)]
                int dims = getNumberOfDimensions();
                double result = 0.0, diff, param;
                for(int j = 0; j < dims; j++) {
                    param = params[j];
                    diff = param - point[j];
                    result += diff * diff;
                    derivatives[j] = 2.0 * diff;
                }

                return result;
            }
        });

        int dims = getNumberOfDimensions();
        double dist;

        try {
            Matrix x = new Matrix(mPositions.length, dims);
            double[] y = new double[mPositions.length];
            for (int i = 0; i < mPositions.length; i++) {
                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, mPositions[i].getInhomogeneousCoordinate(j));
                }

                dist = mDistances[i];
                y[i] =  dist * dist;
            }

            mFitter.setInputData(x, y, mDistanceStandardDeviations);
        } catch (AlgebraException ignore) { }
    }
}
