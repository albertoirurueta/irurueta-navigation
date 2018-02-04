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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.NotReadyException;

/**
 * Solves the trilateration problem.
 * This is a formulation for a nonlinear least squares optimizer.
 */
@SuppressWarnings("WeakerAccess")
public abstract class TrilaterationSolver<P extends Point> {

    /**
     * Minimum allowed distance for a given circle or sphere.
     */
    public static final double EPSILON = 1e-7;

    /**
     * Minimum required number of points to solve trilateration.
     */
    public static final int MIN_POINTS = 2;

    /**
     * Known positions of static nodes.
     */
    protected P[] mPositions;

    /**
     * Euclidean distances from static nodes to mobile node.
     */
    protected double[] mDistances;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected TrilaterationSolverListener<P> mListener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] mEstimatedPositionCoordinates;

    /**
     * Constructor.
     */
    public TrilaterationSolver() { }

    /**
     * Constructor.
     * Sets known positions and euclidean distances.
     * @param positions known positios of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    public TrilaterationSolver(P[] positions, double[] distances) throws IllegalArgumentException {
        setPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    public TrilaterationSolver(TrilaterationSolverListener<P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets known positions and euclidean distances.
     * @param positions known positios of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    public TrilaterationSolver(P[] positions, double[] distances, TrilaterationSolverListener<P> listener)
            throws IllegalArgumentException {
        this(positions, distances);
        mListener = listener;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     * @return listener to be notified of events raised by this instance.
     */
    public TrilaterationSolverListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     * @param listener listener to be notified of events raised by this instance.
     */
    public void setListener(TrilaterationSolverListener<P> listener) {
        mListener = listener;
    }

    /**
     * Gets known positions of static nodes.
     * @return known positions of static nodes.
     */
    public P[] getPositions() {
        return mPositions;
    }

    /**
     * Gets euclidean distances from static nodes to mobile node.
     * @return euclidean distances from static nodes to mobile node.
     */
    public double[] getDistances() {
        return mDistances;
    }

    /**
     * Indicates whether solver is ready to find a solution.
     * @return true if solver is ready, false otherwise.
     */
    public boolean isReady() {
        return mPositions != null && mDistances != null;
    }

    /**
     * Sets known positions and euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     * @param positions known positios of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public void setPositionsAndDistances(P[] positions, double[] distances) throws IllegalArgumentException {
        if(positions == null || distances == null) {
            throw new IllegalArgumentException();
        }

        if (positions.length < MIN_POINTS) {
            throw new IllegalArgumentException();
        }

        if (positions.length != distances.length) {
            throw new IllegalArgumentException();
        }

        mPositions = positions;
        mDistances = distances;

        //fix distances if needed
        for (int i = 0; i < mDistances.length; i++) {
            if (mDistances[i] < EPSILON) {
                mDistances[i] = EPSILON;
            }
        }
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return mEstimatedPositionCoordinates;
    }

    /**
     * Gets estimated estimatedPosition and stores result into provided instance.
     * @param estimatedPosition instance where estimated estimatedPosition will be stored.
     */
    public void getEstimatedPosition(P estimatedPosition) {
        if (mEstimatedPositionCoordinates != null) {
            for (int i = 0; i < mEstimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i,
                        mEstimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets number of dimensions of provided points.
     * @return number of dimensions of provided points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Solves the trilateration problem.
     * @throws TrilaterationException if trilateration fails.
     * @throws NotReadyException is solver is not ready.
     */
    public abstract void solve() throws TrilaterationException, NotReadyException;

    /**
     * Gets trilateration solver type.
     * @return trilateration solver type.
     */
    public abstract TrilaterationSolverType getType();
}
