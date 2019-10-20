package com.irurueta.navigation.frames;

/**
 * Base interface for frames.
 */
public interface Frame {

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    CoordinateTransformationMatrix getCoordinateTransformationMatrix();

    /**
     * Gets coordinate transformation matrix.
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    void getCoordinateTransformationMatrix(final CoordinateTransformationMatrix result);

    /**
     * Sets coordinate transformation matrix.
     * Provided value must be a body to ECEF transformation matrix.
     *
     * @param c coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    void setCoordinateTransformationMatrix(final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException;
}
