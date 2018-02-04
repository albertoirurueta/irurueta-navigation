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
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.NotReadyException;

/**
 * Linearly solves the trilateration problem.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class LinearLeastSquaresTrilaterationSolver<P extends Point> extends TrilaterationSolver<P> {

    /**
     * Constructor.
     */
    public LinearLeastSquaresTrilaterationSolver() {
        super();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required 2 points.
     */
    public LinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances) throws IllegalArgumentException {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    public LinearLeastSquaresTrilaterationSolver(TrilaterationSolverListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required 2 points.
     */
    public LinearLeastSquaresTrilaterationSolver(P[] positions, double[] distances,
            TrilaterationSolverListener<P> listener) throws IllegalArgumentException {
        super(positions, distances, listener);
    }

    /**
     * Solves the trilateration problem.
     * @throws TrilaterationException if trilateration fails.
     * @throws NotReadyException if solver is not ready.
     */
    @Override
    public void solve() throws TrilaterationException, NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            if (mListener != null) {
                mListener.onSolveStart(this);
            }

            int numberOfPositions = mPositions.length;
            int numberOfPositionsMinus1 = numberOfPositions - 1;
            int dims = getNumberOfDimensions();

            Matrix a = new Matrix(numberOfPositionsMinus1, dims);
            for (int i = 1, i2 = 0; i < numberOfPositions; i++, i2++) {
                for (int j = 0; j < dims; j++) {
                    a.setElementAt(i2, j, mPositions[i].getInhomogeneousCoordinate(j) -
                                    mPositions[0].getInhomogeneousCoordinate(j));
                }
            }

            //reference point is first position mPositions[0] with distance mDistances[0]
            double referenceDistance = mDistances[0];
            double sqrRefDistance = referenceDistance * referenceDistance;
            double[] b = new double[numberOfPositionsMinus1];
            for (int i = 1, i2 = 0; i < numberOfPositions; i++, i2++) {
                double ri = mDistances[i];
                double sqrRi = ri * ri;

                //find distance between ri and r0
                //noinspection unchecked
                double sqrRi0 = mPositions[i].sqrDistanceTo(mPositions[0]);
                b[i2] = 0.5 * (sqrRefDistance - sqrRi + sqrRi0);
            }

            mEstimatedPositionCoordinates = Utils.solve(a, b);

            //add position of reference point
            for (int i = 0; i < dims; i++) {
                mEstimatedPositionCoordinates[i] += mPositions[0].getInhomogeneousCoordinate(i);
            }

            if (mListener != null) {
                mListener.onSolveEnd(this);
            }
        } catch (AlgebraException e) {
            throw new TrilaterationException(e);
        }
    }

    /**
     * Gets trilateration solver type.
     * @return trilateration solver type.
     */
    @Override
    public TrilaterationSolverType getType() {
        return TrilaterationSolverType.LINEAR_TRILATERATION_SOLVER;
    }
}
