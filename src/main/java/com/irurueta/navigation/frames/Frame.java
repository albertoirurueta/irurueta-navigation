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
package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Rotation3D;

/**
 * Base interface for frames.
 */
public interface Frame {

    /**
     * Gets coordinate transformation.
     *
     * @return coordinate transformation.
     */
    CoordinateTransformation getCoordinateTransformation();

    /**
     * Gets coordinate transformation.
     *
     * @param result instance where coordinate transformation will be copied to.
     */
    void getCoordinateTransformation(final CoordinateTransformation result);

    /**
     * Sets coordinate transformation.
     * Provided value must be a body to ECEF transformation.
     *
     * @param c coordinate transformation to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    void setCoordinateTransformation(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException;

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient.
     *
     * @return coordinate transformation matrix.
     */
    Matrix getCoordinateTransformationMatrix();

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    void getCoordinateTransformationMatrix(final Matrix result);

    /**
     * Sets coordinate transformation matrix keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting coordinate matrix into copied coordinate transformation and
     * then setting the coordinate transformation calling {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param matrix    a 3x3 coordinate transformation matrix to be set.
     * @param threshold threshold to validate rotation matrix.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     * @throws IllegalArgumentException       if provided threshold is negative.
     */
    void setCoordinateTransformationMatrix(final Matrix matrix, final double threshold)
            throws InvalidRotationMatrixException;

    /**
     * Sts coordinate transformation matrix keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting coordinate matrix into copied coordinate transformation and
     * then setting the coordinate transformation calling {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param matrix a 3x3 coordinate transformation matrix to be set.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     */
    void setCoordinateTransformationMatrix(final Matrix matrix) throws InvalidRotationMatrixException;

    /**
     * Gets coordinate transformation as a new 3D rotation instance.
     * This is equivalent to calling getCoordinateTransformation().asRotation(), but more efficient.
     *
     * @return new coordinate transformation as a 3D rotation.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    Rotation3D getCoordinateTransformationRotation() throws InvalidRotationMatrixException;

    /**
     * Gets coordinate transformation as a 3D rotation.
     * This is equivalent to calling getCoordinateTransformation().asRotation(), but more efficient.
     *
     * @param result instance where coordinate transformation 3D rotation will be copied to.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    void getCoordinateTransformationRotation(final Rotation3D result) throws InvalidRotationMatrixException;

    /**
     * Sets coordinate transformation from 3D rotation and keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting rotation into copied coordinate transformation and
     * then setting the coordinate transformation calling {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param rotation set rotation into current coordinate rotation.
     */
    void setCoordinateTransformationRotation(final Rotation3D rotation);
}
