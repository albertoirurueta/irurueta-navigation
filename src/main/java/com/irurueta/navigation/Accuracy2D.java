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
import com.irurueta.geometry.InvalidRotationMatrixException;

/**
 * Contains methods to convert covariance matrices into ellipses representing accuracy with
 * requested confidence.
 */
@SuppressWarnings("WeakerAccess")
public class Accuracy2D extends Accuracy<com.irurueta.geometry.Accuracy2D> {

    /**
     * Constructor.
     */
    public Accuracy2D() {
        mInternalAccuracy = new com.irurueta.geometry.Accuracy2D();
    }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be 2x2 and positive
     *                         definite.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to an ellipse).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not
     * symmetric and positive definite.
     */
    public Accuracy2D(Matrix covarianceMatrix)
            throws NonSymmetricPositiveDefiniteMatrixException {
        mInternalAccuracy = new com.irurueta.geometry.Accuracy2D(covarianceMatrix);
    }

    /**
     * Constructor.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public Accuracy2D(double confidence) {
        mInternalAccuracy = new com.irurueta.geometry.Accuracy2D(confidence);
    }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be 2x2 and positive
     *                         definite.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to an ellipse), or if provided
     * confidence value is not within 0 and 1.
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not
     * symmetric and positive definite.
     */
    public Accuracy2D(Matrix covarianceMatrix, double confidence)
            throws NonSymmetricPositiveDefiniteMatrixException {
        mInternalAccuracy = new com.irurueta.geometry.Accuracy2D(covarianceMatrix, confidence);
    }

    /**
     * Constructor.
     * @param internalAccuracy internal accuracy to be set.
     */
    Accuracy2D(com.irurueta.geometry.Accuracy2D internalAccuracy) {
        super(internalAccuracy);
    }

    /**
     * Converts provided covariance matrix into a 2D ellipse taking into account current
     * confidence and standard deviation factor.
     * @return ellipse representing accuracy of covariance matrix with current confidence and
     * standard deviation factor.
     * @throws NullPointerException if covariance matrix has not been provided yet.
     * @throws InvalidRotationMatrixException if rotation cannot be properly determined.
     */
    public Ellipse toEllipse() throws InvalidRotationMatrixException {
        return mInternalAccuracy.toEllipse();
    }
}
