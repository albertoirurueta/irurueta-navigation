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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Solves the lateration problem.
 * This is a formulation for a nonlinear least squares optimizer.
 * This class is base on the implementation found at: https://github.com/lemmingapex/trilateration
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("Duplicates")
public abstract class LaterationSolver<P extends Point<?>> {

    /**
     * Minimum allowed distance for a given circle or sphere.
     */
    public static final double EPSILON = 1e-7;

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
    protected LaterationSolverListener<P> mListener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] mEstimatedPositionCoordinates;

    /**
     * Indicates if this instance is locked because lateration is being solved.
     */
    protected boolean mLocked;

    /**
     * Constructor.
     */
    protected LaterationSolver() {
    }

    /**
     * Constructor.
     * Sets known positions and euclidean distances.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    protected LaterationSolver(final P[] positions, final double[] distances) {
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    protected LaterationSolver(final LaterationSolverListener<P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets known positions and euclidean distances.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points).
     */
    protected LaterationSolver(
            final P[] positions, final double[] distances,
            final LaterationSolverListener<P> listener) {
        this(positions, distances);
        mListener = listener;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public LaterationSolverListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    public void setListener(
            final LaterationSolverListener<P> listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets known positions of static nodes.
     *
     * @return known positions of static nodes.
     */
    public P[] getPositions() {
        return mPositions;
    }

    /**
     * Gets euclidean distances from static nodes to mobile node.
     *
     * @return euclidean distances from static nodes to mobile node.
     */
    public double[] getDistances() {
        return mDistances;
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    public boolean isReady() {
        return mPositions != null && mDistances != null &&
                mPositions.length >= getMinRequiredPositionsAndDistances();
    }

    /**
     * Returns boolean indicating if solver is locked because estimation is under
     * progress.
     *
     * @return true if solver is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
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
    public void setPositionsAndDistances(
            final P[] positions, final double[] distances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     *
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return mEstimatedPositionCoordinates;
    }

    /**
     * Gets estimated estimated position and stores result into provided instance.
     *
     * @param estimatedPosition instance where estimated estimated position will be stored.
     */
    public void getEstimatedPosition(final P estimatedPosition) {
        if (mEstimatedPositionCoordinates != null) {
            for (int i = 0; i < mEstimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i,
                        mEstimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets number of dimensions of provided points.
     *
     * @return number of dimensions of provided points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Solves the lateration problem.
     *
     * @throws LaterationException if lateration fails.
     * @throws NotReadyException   is solver is not ready.
     * @throws LockedException     if instance is busy solving the lateration problem.
     */
    public abstract void solve() throws LaterationException, NotReadyException,
            LockedException;

    /**
     * Gets lateration solver type.
     *
     * @return lateration solver type.
     */
    public abstract LaterationSolverType getType();

    /**
     * Minimum required number of positions and distances.
     * This value will depend on actual implementation and whether we are solving a 2D or 3D problem.
     *
     * @return minimum required number of positions and distances.
     */
    public abstract int getMinRequiredPositionsAndDistances();

    /**
     * Internally sets known positions and euclidean distances.
     * If any distance value is zero or negative, it will be fixed assuming an EPSILON value.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    protected void internalSetPositionsAndDistances(
            final P[] positions, final double[] distances) {
        if (positions == null || distances == null) {
            throw new IllegalArgumentException();
        }

        if (positions.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (positions.length != distances.length) {
            throw new IllegalArgumentException();
        }

        mPositions = positions;
        mDistances = distances;

        // fix distances if needed
        for (int i = 0; i < mDistances.length; i++) {
            if (mDistances[i] < EPSILON) {
                mDistances[i] = EPSILON;
            }
        }
    }
}
