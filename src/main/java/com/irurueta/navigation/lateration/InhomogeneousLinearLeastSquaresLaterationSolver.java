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
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Linearly solves the lateration problem using an inhomogeneous solution.
 * This class is base on the implementation found at:
 * <a href="https://github.com/lemmingapex/trilateration">https://github.com/lemmingapex/trilateration</a>
 * Further information and algorithms can be found at Willy Hereman and William S. Murphy Jr. Determination of a
 * Position in Three Dimensions Using Trilateration and Approximate Distances.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class InhomogeneousLinearLeastSquaresLaterationSolver<P extends Point<P>> extends LaterationSolver<P> {

    /**
     * Constructor.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required points.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver(final P[] positions, final double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver(final LaterationSolverListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required points.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver(
            final P[] positions, final double[] distances, final LaterationSolverListener<P> listener) {
        super(positions, distances, listener);
    }

    /**
     * Solves the lateration problem.
     *
     * @throws LaterationException if lateration fails.
     * @throws NotReadyException   if solver is not ready.
     * @throws LockedException     if instance is busy solving the lateration problem.
     */
    @Override
    public void solve() throws LaterationException, NotReadyException, LockedException {
        // The implementation on this method follows the algorithm bellow for 3D but
        // generalized for both 2D and 3D:

        // The constraints are the equations of the spheres with radii ri,
        // (x - xi)^2 + (y - yi)^2 + (z - zi)^2 = ri^2 (i = 1,2,...,n)
        // where (xi, yi, zi) is the center of each sphere and ri is the radius of
        // each sphere.

        // The jth constraint is used as a linearizing tool (in this implementation we
        // always used the 1 point as a reference point). Adding and subtracting xj,yj and zj gives
        // (x - xj + xj - xi)^2 + (y - yj + yj - yi)^2 + (z - zj + zj - zi)^2 = ri^2
        // with (i = 1,2,...j-1,j+1,...,n).

        // Expanding and regrouping the terms, leads to
        // (x - xj)*(xi - xj) + (y - yj)*(yi - yj) + (z - zj)*(zi - zj) =
        // 0.5*((x - xj)^2 + (y - yj)^2 + (z - zj)^2 - ri^2 + (yi - yj)^2 + (zi - zj)^2) =
        // 0.5*(rj^2 - ri^2 + dij^2) = bij
        // where
        // dij = sqrt((xi - xj^2 + (yi - yj^2 + (zi - zj)^2)
        // is the distance between position i and position j.

        // Since it does not matter which constraint is used as a linearizing tool, arbitrarily select
        // the first constraint (j = 1). This is analogous to selecting the first position.
        // Since i = 2,3,...,n, this leads to a linear system of (n - 1) equations in 3 unknowns.
        // (x - x1)*(x2 - x1) + (y - y1)*(y2 - y1) + (z - z1)*(z2 - z1) = 0.5*(r1^2 - r2^2 + d21^2) = b21
        // (x - x1)*(x3 - x1) + (y - y1)*(y3 - y1) + (z - z1)*(z3 - z1) = 0.5*(r1^2 - r3^2 + d31^2) = b31
        // ...
        // (x - x1)*(xn - x1) + (y - y1)*(yn - y1) + (z - z1)*(zn - z1) = 0.5*(r1^2 - rn^2 + dn1^2) = bn1

        // This linear system is easily written in matrix form A*x = b, with
        // A =  [x2 - x1    y2 - y1     z2 - z1],   x = [x - x1],   b = [b21]
        //      [x3 - x1    y3 - y1     z3 - z1]        [y - y1]        [b31]
        //      [...        ...         ...    ]        [z - z1]        [...]
        //      [xn - x1    yn - y1     zn - z1]                        [bn1]

        // Hence, the linear system of equations solves (x - x1, y - y1, z - z1), we need a final step to
        // add the reference point (x1, y1, z1) in order to solve (x, y, z).

        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onSolveStart(this);
            }

            final var numberOfPositions = positions.length;
            final var numberOfPositionsMinus1 = numberOfPositions - 1;
            final var dims = getNumberOfDimensions();

            final var a = new Matrix(numberOfPositionsMinus1, dims);
            for (int i = 1, i2 = 0; i < numberOfPositions; i++, i2++) {
                for (var j = 0; j < dims; j++) {
                    a.setElementAt(i2, j, positions[i].getInhomogeneousCoordinate(j)
                            - positions[0].getInhomogeneousCoordinate(j));
                }
            }

            // reference point is first position mPositions[0] with distance mDistances[0]
            final var referenceDistance = distances[0];
            final var sqrRefDistance = referenceDistance * referenceDistance;
            final var b = new double[numberOfPositionsMinus1];
            for (int i = 1, i2 = 0; i < numberOfPositions; i++, i2++) {
                final var ri = distances[i];
                final var sqrRi = ri * ri;

                // find distance between ri and r0
                final var sqrRi0 = positions[i].sqrDistanceTo(positions[0]);
                b[i2] = 0.5 * (sqrRefDistance - sqrRi + sqrRi0);
            }

            estimatedPositionCoordinates = Utils.solve(a, b);

            // add position of reference point
            for (var i = 0; i < dims; i++) {
                estimatedPositionCoordinates[i] += positions[0].getInhomogeneousCoordinate(i);
            }

            if (listener != null) {
                listener.onSolveEnd(this);
            }
        } catch (final AlgebraException e) {
            throw new LaterationException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Gets lateration solver type.
     *
     * @return lateration solver type.
     */
    @Override
    public LaterationSolverType getType() {
        return LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER;
    }
}
