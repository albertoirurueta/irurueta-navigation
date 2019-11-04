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
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient
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
}
