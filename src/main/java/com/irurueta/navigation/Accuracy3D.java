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
package com.irurueta.navigation;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Ellipse;
import com.irurueta.geometry.Ellipsoid;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.InvalidRotationMatrixException;

/**
 * Contains methods to convert covariance matrices into ellipsoids representing accuracy with
 * requested confidence.
 */
public class Accuracy3D extends Accuracy<com.irurueta.geometry.Accuracy3D> {

    /**
     * Constructor.
     */
    public Accuracy3D() {
        internalAccuracy = new com.irurueta.geometry.Accuracy3D();
    }

    /**
     * Constructor.
     *
     * @param covarianceMatrix covariance matrix to be set. Must be 3x3 and positive
     *                         definite.
     * @throws IllegalArgumentException                    if provided matrix is not square (it must also
     *                                                     be positive definite to be properly converted to an
     *                                                     ellipsoid).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not
     *                                                     symmetric and positive definite.
     */
    public Accuracy3D(final Matrix covarianceMatrix) throws NonSymmetricPositiveDefiniteMatrixException {
        internalAccuracy = new com.irurueta.geometry.Accuracy3D(covarianceMatrix);
    }

    /**
     * Constructor.
     *
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public Accuracy3D(final double confidence) {
        internalAccuracy = new com.irurueta.geometry.Accuracy3D(confidence);
    }

    /**
     * Constructor.
     *
     * @param covarianceMatrix covariance matrix to be set. Must be 3x3 and positive
     *                         definite.
     * @param confidence       confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException                    if provided matrix is not square (it must also
     *                                                     be positive definite to be properly converted to an
     *                                                     ellipsoid), or if provided confidence value is not within 0
     *                                                     and 1.
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix it not
     *                                                     symmetric and positive definite.
     */
    public Accuracy3D(final Matrix covarianceMatrix, final double confidence)
            throws NonSymmetricPositiveDefiniteMatrixException {
        internalAccuracy = new com.irurueta.geometry.Accuracy3D(covarianceMatrix, confidence);
    }

    /**
     * Constructor.
     *
     * @param internalAccuracy internal accuracy to be set.
     */
    Accuracy3D(final com.irurueta.geometry.Accuracy3D internalAccuracy) {
        super(internalAccuracy);
    }

    /**
     * Converts provided covariance matrix into a 3D ellipsoid taking into account current
     * confidence and standard deviation factor.
     *
     * @return ellipsoid representing accuracy of covariance matrix with current confidence and
     * standard deviation factor.
     * @throws NullPointerException           if covariance matrix has not been provided yet.
     * @throws InvalidRotationMatrixException if rotation cannot be properly determined.
     */
    public Ellipsoid toEllipsoid() throws InvalidRotationMatrixException {
        return internalAccuracy.toEllipsoid();
    }

    /**
     * Flattens accuracy representation to 2D by taking into account only x and y coordinates and
     * ignoring variance related to z coordinates.
     *
     * @return flattened accuracy representation in 2D.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws GeometryException    if accuracy cannot be flattened.
     */
    public Accuracy2D flattenTo2D() throws GeometryException {
        return new Accuracy2D(internalAccuracy.flattenTo2D());
    }

    /**
     * Intersects ellipsoid representing this accuracy with horizontal xy plane.
     *
     * @return intersected ellipse.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws GeometryException    if intersection cannot be computed.
     */
    public Ellipse intersectWithPlane() throws GeometryException {
        return internalAccuracy.intersectWithPlane();
    }
}
