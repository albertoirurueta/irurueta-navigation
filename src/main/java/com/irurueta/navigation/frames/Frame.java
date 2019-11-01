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
