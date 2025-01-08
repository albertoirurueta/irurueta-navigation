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
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;

/**
 * Base class representing the confidence of provided accuracy from a covariance matrix
 * expressed in the distance unit of such matrix.
 * This class contains utility methods to convert covariance matrices into geometric figures
 * with the requested confidence.
 *
 * @param <A> type of internal accuracy.
 */
public abstract class Accuracy<A extends com.irurueta.geometry.Accuracy> {

    /**
     * Internal accuracy reference.
     */
    protected A internalAccuracy;

    /**
     * Constructor.
     */
    protected Accuracy() {
    }

    /**
     * Constructor.
     *
     * @param internalAccuracy internal accuracy to be set.
     */
    Accuracy(final A internalAccuracy) {
        this.internalAccuracy = internalAccuracy;
    }

    /**
     * Gets covariance matrix representing the accuracy of an estimated point or measure.
     *
     * @return covariance matrix representing the accuracy of an estimated point or measure.
     */
    public Matrix getCovarianceMatrix() {
        return internalAccuracy.getCovarianceMatrix();
    }

    /**
     * Sets covariance matrix representing the accuracy of an estimated point or measure.
     *
     * @param covarianceMatrix covariance matrix representing the accuracy of an estimated
     *                         point or measure.
     * @throws IllegalArgumentException                    if provided matrix is not square (it must also be
     *                                                     positive definite to be properly converted to a geometric
     *                                                     figure - e.g. an ellipse or an ellipsoid).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric
     *                                                     and positive definite.
     */
    public void setCovarianceMatrix(final Matrix covarianceMatrix) throws NonSymmetricPositiveDefiniteMatrixException {
        internalAccuracy.setCovarianceMatrix(covarianceMatrix);
    }

    /**
     * Gets standard deviation factor to account for a given accuracy confidence.
     * Typically, a factor of 2.0 will be used, which means that accuracy can be drawn as
     * a geometric figure of size equal to 2 times the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% confidence on provided
     * accuracy.
     *
     * @return standard deviation factor.
     */
    public double getStandardDeviationFactor() {
        return internalAccuracy.getStandardDeviationFactor();
    }

    /**
     * Sets standard deviation factor to account for a given accuracy confidence.
     * Typically, a factor of 2.0 will be used, which means that accuracy can be drawn as
     * a geometric figure of size equal to 2 times the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% confidence on provided
     * accuracy.
     *
     * @param standardDeviationFactor standard deviation factor to be set.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setStandardDeviationFactor(final double standardDeviationFactor) {
        internalAccuracy.setStandardDeviationFactor(standardDeviationFactor);
    }

    /**
     * Gets confidence of provided accuracy of estimated point or measure.
     * This is expressed as a value between 0 and 1, where 1 indicates a 100% confidence
     * that the real point or measure is within provided accuracy.
     *
     * @return confidence of provided accuracy of estimated point or measure.
     */
    public double getConfidence() {
        return internalAccuracy.getConfidence();
    }

    /**
     * Sets confidence of provided accuracy of estimated point or measure.
     * This is expressed as a value between 0 and 1, where 1 indicates a 100% confidence
     * that the real point or measure is within provided accuracy.
     *
     * @param confidence confidence of provided accuracy of estimated point or measure.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public void setConfidence(final double confidence) {
        internalAccuracy.setConfidence(confidence);
    }

    /**
     * Gets smallest (best) accuracy in any direction (i.e. either 2D or 3D).
     * This value is represented by the smallest semi axis representing the ellipse or ellipsoid of accuracy.
     *
     * @return smallest accuracy in any direction.
     */
    public Distance getSmallestAccuracy() {
        return new Distance(getSmallestAccuracyMeters(), DistanceUnit.METER);
    }

    /**
     * Gets smallest (best) accuracy in any direction (i.e. either 2D or 3D)
     * expressed in meters.
     * This value is represented by the smallest semi axis representing the ellipse or ellipsoid of accuracy.
     *
     * @return smallest accuracy in any direction expressed in meters.
     */
    public double getSmallestAccuracyMeters() {
        return internalAccuracy.getSmallestAccuracy();
    }

    /**
     * Gets largest (worse) accuracy in any direction (i.e. either 2D or 3D).
     * This value is represented by the largest semi axis representing the ellipse or ellipsoid of accuracy.
     *
     * @return largest accuracy in any direction.
     */
    public Distance getLargestAccuracy() {
        return new Distance(getLargestAccuracyMeters(), DistanceUnit.METER);
    }

    /**
     * Gets largest (worse) accuracy in any direction (i.e. either 2D or 3D)
     * expressed in meters.
     * This value is represented by the largest semi axis representing the ellipse or ellipsoid of accuracy.
     *
     * @return largest accuracy in any direction expressed in meters.
     */
    public double getLargestAccuracyMeters() {
        return internalAccuracy.getLargestAccuracy();
    }

    /**
     * Gets average accuracy among all directions.
     * This value is equal to the average value of all semi axes representing the ellipse or ellipsoid of
     * accuracy.
     *
     * @return average accuracy among all directions.
     */
    public Distance getAverageAccuracy() {
        return new Distance(getAverageAccuracyMeters(), DistanceUnit.METER);
    }

    /**
     * Gets average accuracy among all directions expressed in meters.
     * This value is equal to the average value of all semi axes representing the ellipse or ellipsoid of
     * accuracy.
     *
     * @return average accuracy among all directions expressed in meters.
     */
    public double getAverageAccuracyMeters() {
        return internalAccuracy.getAverageAccuracy();
    }

    /**
     * Gets number of dimensions.
     * This is equal to 2 for 2D, and 3 for 3D.
     *
     * @return number of dimensions.
     */
    public int getNumberOfDimensions() {
        return internalAccuracy.getNumberOfDimensions();
    }
}
