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
package com.irurueta.navigation.lateration;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Linearly solves the lateration problem using an homogeneous LMSE solution.
 */
@SuppressWarnings("WeakerAccess")
public abstract class HomogeneousLinearLeastSquaresLaterationSolver<P extends Point> extends
        LaterationSolver<P> {

    /**
     * Constructor.
     */
    public HomogeneousLinearLeastSquaresLaterationSolver() {
        super();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required points.
     */
    public HomogeneousLinearLeastSquaresLaterationSolver(P[] positions, double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    public HomogeneousLinearLeastSquaresLaterationSolver(LaterationSolverListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either position or distances are null,
     * don't have the same length or their length is smaller than required points.
     */
    public HomogeneousLinearLeastSquaresLaterationSolver(P[] positions, double[] distances,
                                                         LaterationSolverListener<P> listener) {
        super(positions, distances, listener);
    }

    /**
     * Solves the lateration problem.
     * @throws LaterationException if lateration fails.
     * @throws NotReadyException if solver is not ready.
     * @throws LockedException if instance is busy solving the lateration problem.
     */
    @Override
    @SuppressWarnings("Duplicates")
    public void solve() throws LaterationException, NotReadyException,
            LockedException {
        // The implementation on this method follows the algorithm  bellow.

        // Having 3 2D circles:
        // c1x, c1y, r1
        // c2x, c2y, r2
        // c3x, c3y, r3
        // where (c1x, c1y) are the coordinates of 1st circle center and r1 is its radius.
        // (c2x, c2y) are the coordinates of 2nd circle center and r2 is its radius.
        // (c3x, c3y) are the coordinates of 3rd circle center and r3 is its radius.

        // The equations of the circles are as follows:
        // (x - c1x)^2 + (y - c1y)^2 = r1^2
        // (x - c2x)^2 + (y - c2y)^2 = r2^2
        // (x - c3x)^2 + (y - c3y)^2 = r3^2

        // x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2 = r1^2
        // x^2 - 2*c2x*x + c2x^2 + y^2 - 2*c2y*y + c2y^2 = r2^2
        // x^2 - 2*c3x*x + c3x^2 + y^2 - 2*c3y*y + c3y^2 = r3^2

        // remove 1st equation from others (we use 1st point as reference)

        // x^2 - 2*c2x*x + c2x^2 + y^2 - 2*c2y*y + c2y^2 - (x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2) = r2^2 - r1^2
        // x^2 - 2*c3x*x + c3x^2 + y^2 - 2*c3y*y + c3y^2 - (x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2) = r3^2 - r1^2

        // - 2*c2x*x + c2x^2 - 2*c2y*y + c2y^2 + 2*c1x*x - c1x^2 + 2*c1y*y - c1y^2 = r2^2 - r1^2
        // - 2*c3x*x + c3x^2 - 2*c3y*y + c3y^2 + 2*c1x*x - c1x^2 + 2*c1y*y - c1y^2 = r3^2 - r1^2

        // 2*(c1x - c2x)*x + c2x^2 + 2*(c1y - c2y)*y + c2y^2 - c1x^2 - c1y^2 = r2^2 - r1^2
        // 2*(c1x - c3x)*x + c3x^2 + 2*(c1y - c3y)*y + c3y^2 - c1x^2 - c1y^2 = r3^2 - r1^2

        // 2*(c1x - c2x)*x + 2*(c1y - c2y)*y = r2^2 - r1^2 + c1x^2 + c1y^2 - c2x^2 - c2y^2
        // 2*(c1x - c3x)*x + 2*(c1y - c3y)*y = r3^2 - r1^2 + c1x^2 + c1y^2 - c3x^2 - c3y^2

        // x and y are the inhomogeneous coordinates of the point (x,y) we want to find, we
        // substitute such point by the corresponding homogeneous coordinates (x,y) = (x'/w', y'/w')

        // Hence
        // 2*(c1x - c2x)*x'/w' + 2*(c1y - c2y)*y'/w' = r2^2 - r1^2 + c1x^2 + c1y^2 - c2x^2 - c2y^2
        // 2*(c1x - c3x)*x'/w' + 2*(c1y - c3y)*y'/w' = r3^2 - r1^2 + c1x^2 + c1y^2 - c3x^2 - c3y^2

        // Multiplitying by w' at both sides...
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' = (r2^2 - r1^2 + c1x^2 + c1y^2 - c2x^2 - c2y^2)*w'
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' = (r3^2 - r1^2 + c1x^2 + c1y^2 - c3x^2 - c3y^2)*w'

        // Obtaining the following homogeneous equations
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' - (r2^2 - r1^2 + c1x^2 + c1y^2 - c2x^2 - c2y^2)*w' = 0
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' - (r3^2 - r1^2 + c1x^2 + c1y^2 - c3x^2 - c3y^2)*w' = 0

        // Fixing signs...
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' + (r1^2 - r2^2 + c2x^2 + c2y^2 - c1x^2 - c1y^2)*w' = 0
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' + (r1^2 - r3^2 + c3x^2 + c3y^2 - c1x^2 - c1y^2)*w' = 0


        // The homogeneous equations can be expressed as a linear system of homogeneous equations A*x = 0
        // where the unknowns to be solved are (x', y', w') up to scale.

        // [2*(c1x - c2x)   2*(c1y - c2y)    r1^2 - r2^2 + c2x^2 + c2y^2 - c1x^2 - c1y^2][x'] = 0
        // [2*(c1x - c3x)   2*(c1y - c3y)    r1^2 - r3^2 + c3x^2 + c3y^2 - c1x^2 - c1y^2][y'] = 0
        //                                                                               [w']

        // This can be solved by using the SVD decomposition of matrix A and picking the last column of
        // resulting V matrix. At least 2 equations are required to find a solution, since 1 additional
        // point is used as a reference, at least 3 points are required.

        // For spheres the solution is analogous

        // Having 4 3D spheres:
        // c1x, c1y, c1z, r1
        // c2x, c2y, c2z, r2
        // c3x, c3y, c3z, r3
        // c4x, c4y, c4z, r4
        // where (c1x, c1y, c1z) are the coordinates of 1st sphere center and r1 is its radius.
        // (c2x, c2y, c2z) are the coordinates of 2nd sphere center and r2 is its radius.
        // (c3x, c3y, c3z) are the coordinates of 3rd sphere center and r3 is its radius.
        // (c4x, c4y, c4z) are the coordinates of 4th sphere center and r4 is its radius.

        // The equations of the spheres are as follows:
        // (x - c1x)^2 + (y - c1y)^2 + (z - c1z)^2 = r1^2
        // (x - c2x)^2 + (y - c2y)^2 + (z - c2z)^2 = r2^2
        // (x - c3x)^2 + (y - c3y)^2 + (z - c3z)^2 = r3^2
        // (x - c4x)^2 + (y - c4y)^2 + (z - c4z)^2 = r4^2

        // x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2 + z^2 - 2*c1z*z + c1z^2 = r1^2
        // x^2 - 2*c2x*x + c2x^2 + y^2 - 2*c2y*y + c2y^2 + z^2 - 2*c2z*z + c2z^2 = r2^2
        // x^2 - 2*c3x*x + c3x^2 + y^2 - 2*c3y*y + c3y^3 + z^2 - 2*c3z*z + c3z^2 = r3^2
        // x^2 - 2*c4x*x + c4x^2 + y^2 - 2*c4y*y + c4y^2 + z^2 - 2*c4z*z + c4z^2 = r4^2

        // remove 1st equation from others (we use 1st point as reference)
        // x^2 - 2*c2x*x + c2x^2 + y^2 - 2*c2y*y + c2y^2 + z^2 - 2*c2z*z + c2z^2 - (x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2 + z^2 - 2*c1z*z + c1z^2) = r2^2 - r1^2
        // x^2 - 2*c3x*x + c3x^2 + y^2 - 2*c3y*y + c3y^3 + z^2 - 2*c3z*z + c3z^2 - (x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2 + z^2 - 2*c1z*z + c1z^2) = r3^2 - r1^2
        // x^2 - 2*c4x*x + c4x^2 + y^2 - 2*c4y*y + c4y^2 + z^2 - 2*c4z*z + c4z^2 - (x^2 - 2*c1x*x + c1x^2 + y^2 - 2*c1y*y + c1y^2 + z^2 - 2*c1z*z + c1z^2) = r4^2 - r1^2

        // - 2*c2x*x + c2x^2 - 2*c2y*y + c2y^2 - 2*c2z*z + c2z^2 + 2*c1x*x - c1x^2 + 2*c1y*y - c1y^2 + 2*c1z*z - c1z^2 = r2^2 - r1^2
        // - 2*c3x*x + c3x^2 - 2*c3y*y + c3y^3 - 2*c3z*z + c3z^2 + 2*c1x*x - c1x^2 + 2*c1y*y - c1y^2 + 2*c1z*z - c1z^2 = r3^2 - r1^2
        // - 2*c4x*x + c4x^2 - 2*c4y*y + c4y^2 - 2*c4z*z + c4z^2 + 2*c1x*x - c1x^2 + 2*c1y*y - c1y^2 + 2*c1z*z - c1z^2 = r4^2 - r1^2

        // 2*(c1x - c2x)*x + c2x^2 + 2*(c1y - c2y)*y + c2y^2 + 2*(c1z - c2z)*z + c2z^2 - c1x^2 - c1y^2 - c1z^2 = r2^2 - r1^2
        // 2*(c1x - c3x)*x + c3x^2 + 2*(c1y - c3y)*y + c3y^3 + 2*(c1z - c3z)*z + c3z^2 - c1x^2 - c1y^2 - c1z^2 = r3^2 - r1^2
        // 2*(c1x - c4x)*x + c4x^2 + 2*(c1y - c4y)*y + c4y^2 + 2*(c1z - c4z)*z + c4z^2 - c1x^2 - c1y^2 - c1z^2 = r4^2 - r1^2

        // 2*(c1x - c2x)*x + 2*(c1y - c2y)*y + 2*(c1z - c2z)*z = r2^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c2x^2 - c2y^2 - c2z^2
        // 2*(c1x - c3x)*x + 2*(c1y - c3y)*y + 2*(c1z - c3z)*z = r3^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c3x^2 - c3y^3 - c3z^2
        // 2*(c1x - c4x)*x + 2*(c1y - c4y)*y + 2*(c1z - c4z)*z = r4^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c4x^2 - c4y^2 - c4z^2

        // x, y and z the inhomogeneous coordinates of the point (x,y,z) we want to find,
        // we substitute such point by the corresponding homogeneous coordinates
        // (x, y, z) = (x'/w', y'/w', z'/w')

        // Hence
        // 2*(c1x - c2x)*x'/w' + 2*(c1y - c2y)*y'/w' + 2*(c1z - c2z)*z'/w' = r2^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c2x^2 - c2y^2 - c2z^2
        // 2*(c1x - c3x)*x'/w' + 2*(c1y - c3y)*y'/w' + 2*(c1z - c3z)*z'/w' = r3^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c3x^2 - c3y^3 - c3z^2
        // 2*(c1x - c4x)*x'/w' + 2*(c1y - c4y)*y'/w' + 2*(c1z - c4z)*z'/w' = r4^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c4x^2 - c4y^2 - c4z^2

        // Multipliying by w' at both sides...
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' + 2*(c1z - c2z)*z' = (r2^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c2x^2 - c2y^2 - c2z^2)*w'
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' + 2*(c1z - c3z)*z' = (r3^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c3x^2 - c3y^3 - c3z^2)*w'
        // 2*(c1x - c4x)*x' + 2*(c1y - c4y)*y' + 2*(c1z - c4z)*z' = (r4^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c4x^2 - c4y^2 - c4z^2)*w'

        // Obtaining the following homogeneous equations
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' + 2*(c1z - c2z)*z' - (r2^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c2x^2 - c2y^2 - c2z^2)*w' = 0
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' + 2*(c1z - c3z)*z' - (r3^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c3x^2 - c3y^3 - c3z^2)*w' = 0
        // 2*(c1x - c4x)*x' + 2*(c1y - c4y)*y' + 2*(c1z - c4z)*z' - (r4^2 - r1^2 + c1x^2 + c1y^2 + c1z^2 - c4x^2 - c4y^2 - c4z^2)*w' = 0

        // Fixing signs...
        // 2*(c1x - c2x)*x' + 2*(c1y - c2y)*y' + 2*(c1z - c2z)*z' + (r1^2 - r2^2 + c2x^2 + c2y^2 + c2z^2 - c1x^2 - c1y^2 - c1z^2)*w' = 0
        // 2*(c1x - c3x)*x' + 2*(c1y - c3y)*y' + 2*(c1z - c3z)*z' + (r1^2 - r3^2 + c3x^2 + c3y^3 + c3z^2 - c1x^2 - c1y^2 - c1z^2)*w' = 0
        // 2*(c1x - c4x)*x' + 2*(c1y - c4y)*y' + 2*(c1z - c4z)*z' + (r1^2 - r4^2 + c4x^2 + c4y^2 + c4z^2 - c1x^2 - c1y^2 - c1z^2)*w' = 0

        // The homogeneous equastions can be expressed as a linear system of homogeneous equations
        // where the unknowns to be solved are (x', y', z', w') up to scale.

        // [2*(c1x - c2x)   2*(c1y - c2y)   2*(c1z - c2z)   r1^2 - r2^2 + c2x^2 + c2y^2 + c2z^2 - c1x^2 - c1y^2 - c1z^2][x'] = 0
        // [2*(c1x - c3x)   2*(c1y - c3y)   2*(c1z - c3z)   r1^2 - r3^2 + c3x^2 + c3y^3 + c3z^2 - c1x^2 - c1y^2 - c1z^2][y'] = 0
        // [2*(c1x - c4x)   2*(c1y - c4y)   2*(c1z - c4z)   r1^2 - r4^2 + c4x^2 + c4y^2 + c4z^2 - c1x^2 - c1y^2 - c1z^2][z'] = 0
        //                                                                                                              [w']

        // This can be solved by using the SVD decomposition of matrix A and picking the last column of
        // resulting V matrix. At least 3 equations are required to find a solution, since 1 additional
        // point is used as a reference, at least 4 points are required.

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

            int numberOfPositions = mPositions.length;
            int numberOfPositionsMinus1 = numberOfPositions - 1;
            int dims = getNumberOfDimensions();

            double referenceDistance = mDistances[0];
            double sqrRefDistance = referenceDistance * referenceDistance;
            P refPosition = mPositions[0];

            double sqrRefNorm = 0.0;
            for (int j = 0; j < dims; j++) {
                double coord = refPosition.getInhomogeneousCoordinate(j);
                sqrRefNorm += coord * coord;
            }

            Matrix a = new Matrix(numberOfPositionsMinus1, dims + 1);
            for (int i = 1, i2 = 0; i < numberOfPositions; i++, i2++) {

                double sqrNorm = 0.0;
                for (int j = 0; j < dims; j++) {
                    double refCoord = refPosition.getInhomogeneousCoordinate(j);
                    double coord = mPositions[i].getInhomogeneousCoordinate(j);

                    a.setElementAt(i2, j, 2.0 * (refCoord - coord));

                    sqrNorm += coord * coord;
                }

                double distance = mDistances[i];
                double sqrDistance = distance * distance;
                a.setElementAt(i2, dims, sqrRefDistance - sqrDistance +
                        sqrNorm - sqrRefNorm);
            }

            SingularValueDecomposer decomponer = new SingularValueDecomposer(a);
            decomponer.decompose();

            int nullity = decomponer.getNullity();
            if (nullity > 1) {
                // linear system of equations is degenerate (does not have enough rank),
                // probably because there are dependencies or repeated data between
                // points
                throw new LaterationException();
            }

            Matrix v = decomponer.getV();
            double[] homogeneousEstimatedPositionCoordinates = v.getSubmatrixAsArray(
                    0, dims, dims, dims);

            double w = homogeneousEstimatedPositionCoordinates[dims];
            mEstimatedPositionCoordinates = new double[dims];
            for (int j = 0; j < dims; j++) {
                mEstimatedPositionCoordinates[j] =
                        homogeneousEstimatedPositionCoordinates[j] / w;
            }

            if (mListener != null) {
                mListener.onSolveEnd(this);
            }
        } catch (AlgebraException e) {
            throw new LaterationException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets lateration solver type.
     * @return lateration solver type.
     */
    @Override
    public LaterationSolverType getType() {
        return LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER;
    }
}
