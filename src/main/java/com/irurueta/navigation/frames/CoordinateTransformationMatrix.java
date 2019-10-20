package com.irurueta.navigation.frames;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.Rotation3D;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a coordinate transformation matrix, or rotation matrix. The coordinate transformation matrix is a 3x3
 * matrix where a vector may be transformed in one step from one set of resolving axes to another by pre-multiplying it
 * by the appropriate coordinate transformation matrix. The coordinate transformation matrix is associated to a source
 * and destination frame.
 */
@SuppressWarnings("WeakerAccess")
public class CoordinateTransformationMatrix implements Serializable, Cloneable {

    /**
     * Number of rows of a coordinate transformation matrix.
     */
    public static final int ROWS = MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS;

    /**
     * Number of columns of a coordinate transformation matrix.
     */
    public static final int COLS = MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS;

    /**
     * Default threshold to consider a matrix valid.
     */
    public static final double DEFAULT_THRESHOLD = Rotation3D.DEFAULT_VALID_THRESHOLD;

    /**
     * 3x3 matrix containing a rotation.
     */
    private Matrix mMatrix;

    /**
     * Source frame type.
     */
    private FrameType mSourceType;

    /**
     * Destination frame type.
     */
    private FrameType mDestinationType;

    /**
     * Constructor.
     * Initializes rotation as the identify (no rotation).
     *
     * @param sourceType      source frame type.
     * @param destinationType destination frame type.
     * @throws NullPointerException if either source or destination frame types are null.
     */
    public CoordinateTransformationMatrix(final FrameType sourceType, final FrameType destinationType) {
        try {
            mMatrix = Matrix.identity(ROWS, COLS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        setSourceType(sourceType);
        setDestinationType(destinationType);
    }

    /**
     * Constructor.
     *
     * @param matrix          a 3x3 matrix containing a rotation.
     * @param sourceType      source frame type.
     * @param destinationType destination frame type.
     * @param threshold       threshold to validate rotation matrix.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     * @throws NullPointerException           if either source or destination frame types are null.
     * @throws IllegalArgumentException       if provided threshold is negative.
     */
    public CoordinateTransformationMatrix(final Matrix matrix, final FrameType sourceType,
                                          final FrameType destinationType,
                                          final double threshold) throws InvalidRotationMatrixException {
        setMatrix(matrix, threshold);
        setSourceType(sourceType);
        setDestinationType(destinationType);
    }

    /**
     * Constructor.
     *
     * @param matrix          a 3x3 matrix containing a rotation.
     * @param sourceType      source frame type.
     * @param destinationType destination frame type.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     * @throws NullPointerException           if either source or destination frame types are null.
     */
    public CoordinateTransformationMatrix(final Matrix matrix, final FrameType sourceType,
                                          final FrameType destinationType) throws InvalidRotationMatrixException {
        this(matrix, sourceType, destinationType, DEFAULT_THRESHOLD);
    }

    /**
     * Constructor.
     *
     * @param input other coordinate transformation matrix to copy data from.
     */
    public CoordinateTransformationMatrix(final CoordinateTransformationMatrix input) {
        this(input.mSourceType, input.mDestinationType);
        copyFrom(input);
    }

    /**
     * Gets matrix containing a rotation.
     *
     * @return 3x3 matrix containing a rotation.
     */
    public Matrix getMatrix() {
        Matrix result;
        try {
            result = new Matrix(ROWS, COLS);
            getMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets matrix containing a rotation.
     *
     * @param result instance where internal 3x3 matrix containing a rotation will be copied to.
     */
    public void getMatrix(Matrix result) {
        mMatrix.copyTo(result);
    }

    /**
     * Sets matrix containing a rotation.
     *
     * @param matrix    a 3x3 matrix containing a rotation.
     * @param threshold threshold to validate rotation matrix.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     * @throws IllegalArgumentException       if provided threshold is negative.
     */
    public void setMatrix(final Matrix matrix, final double threshold) throws InvalidRotationMatrixException {

        if (!isValidMatrix(matrix, threshold)) {
            throw new InvalidRotationMatrixException();
        }

        mMatrix = matrix;
    }

    /**
     * Sets matrix containing a rotation.
     *
     * @param matrix a 3x3 matrix containing a rotation.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     */
    public void setMatrix(final Matrix matrix) throws InvalidRotationMatrixException {
        setMatrix(matrix, Rotation3D.DEFAULT_VALID_THRESHOLD);
    }

    /**
     * Determines whether provided matrix is a valid rotation matrix (3x3 and orthonormal)
     * up to provided threshold.
     *
     * @param matrix    matrix to be checked.
     * @param threshold threshold to determine whether matrix is valid.
     * @return true if matrix is valid, false otherwise.
     * @throws IllegalArgumentException if provided threshold value is negative.
     */
    public static boolean isValidMatrix(final Matrix matrix, final double threshold) {
        if (threshold < Rotation3D.MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            //TODO: use Rotation3D.isValidRotationMatrix when geometry library is updated
            // instead of Utils.isOrthogonal and checking determinant (note that size must also
            // be checked anyways)
            return matrix.getRows() == Rotation3D.INHOM_COORDS
                    && matrix.getColumns() == Rotation3D.INHOM_COORDS
                    && Utils.isOrthogonal(matrix, threshold) &&
                    Math.abs(Math.abs(Utils.det(matrix)) - 1.0) < threshold;
        } catch (final AlgebraException e) {
            return false;
        }
    }

    /**
     * Determines whether provided matrix is a valid rotation matrix (3x3 and orthonormal)
     * up to default threshold {@link #DEFAULT_THRESHOLD}.
     *
     * @param matrix matrix to be checked.
     * @return true if matrix is valid, false otherwise.
     */
    public static boolean isValidMatrix(final Matrix matrix) {
        return isValidMatrix(matrix, DEFAULT_THRESHOLD);
    }

    /**
     * Gets source frame type.
     *
     * @return source frame type.
     */
    public FrameType getSourceType() {
        return mSourceType;
    }

    /**
     * Sets source frame type.
     *
     * @param sourceType source frame type.
     * @throws NullPointerException if provided value is null.
     */
    public void setSourceType(final FrameType sourceType) {
        if (sourceType == null) {
            throw new NullPointerException();
        }

        mSourceType = sourceType;
    }

    /**
     * Gets destination frame type.
     *
     * @return destination frame type.
     */
    public FrameType getDestinationType() {
        return mDestinationType;
    }

    /**
     * Sets destination frame type.
     *
     * @param destinationType destination frame type.
     * @throws NullPointerException if provided value is null.
     */
    public void setDestinationType(final FrameType destinationType) {
        if (destinationType == null) {
            throw new NullPointerException();
        }

        mDestinationType = destinationType;
    }

    /**
     * Gets internal matrix as a 3D rotation.
     *
     * @return a 3D rotation representing the internal matrix.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    public Rotation3D asRotation() throws InvalidRotationMatrixException {
        return new MatrixRotation3D(mMatrix);
    }

    /**
     * Gets internal matrix as a 3D rotation.
     *
     * @param result instance where 3D rotation will be stored.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    public void asRotation(final Rotation3D result) throws InvalidRotationMatrixException {
        result.fromMatrix(mMatrix);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final CoordinateTransformationMatrix output) {
        output.mSourceType = mSourceType;
        output.mDestinationType = mDestinationType;
        mMatrix.copyTo(output.mMatrix);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final CoordinateTransformationMatrix input) {
        mSourceType = input.mSourceType;
        mDestinationType = input.mDestinationType;
        mMatrix.copyFrom(input.mMatrix);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mSourceType, mDestinationType, mMatrix);
    }

    /**
     * Checks if provided object is a CoordinateTransformationMatrix having exactly the same
     * contents as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof CoordinateTransformationMatrix)) {
            return false;
        }

        final CoordinateTransformationMatrix other = (CoordinateTransformationMatrix) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final CoordinateTransformationMatrix other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to
     * provided threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between values on internal matrix.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final CoordinateTransformationMatrix other, final double threshold) {
        if (other == null) {
            return false;
        }
        return other.mSourceType == mSourceType && other.mDestinationType == mDestinationType &&
                other.mMatrix.equals(mMatrix, threshold);
    }

    /**
     * Computes the inverse of this coordinate transformation matrix and stores the result into provided instance.
     *
     * @param result instance where inverse will be stored.
     */
    public void inverse(final CoordinateTransformationMatrix result) {
        try {
            final FrameType source = mSourceType;
            final FrameType destination = mDestinationType;
            final Matrix matrix = Matrix.identity(ROWS, COLS);
            matrix.copyFrom(mMatrix);

            result.setSourceType(destination);
            result.setDestinationType(source);

            // Because matrix needs to be a rotation (3x3 and orthonormal), its inverse is the transpose
            matrix.transpose();
            result.setMatrix(matrix);
        } catch (WrongSizeException | InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Converts this instance into its inverse coordinate transformation matrix.
     */
    public void inverse() {
        inverse(this);
    }

    /**
     * Computes the inverse of this coordinate transformation matrix and returns it as a new instance.
     *
     * @return the inverse of this coordinate transformation matrix.
     */
    public CoordinateTransformationMatrix inverseAndReturnNew() {
        final CoordinateTransformationMatrix result = new CoordinateTransformationMatrix(mDestinationType, mSourceType);
        inverse(result);
        return result;
    }

    /**
     * Computes matrix to convert ECEF to NED coordinates.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where computed matrix will be stored.
     */
    public static void ecefToNedMatrix(final double latitude, final double longitude, final Matrix result) {
        if (result.getRows() != ROWS || result.getColumns() != COLS) {
            try {
                result.resize(ROWS, COLS);
            } catch (final WrongSizeException ignore) {
            }
        }
        // Calculate ECEF to NED coordinate transformation matrix using (2.150)
        final double cosLat = Math.cos(latitude);
        final double sinLat = Math.sin(latitude);
        final double cosLong = Math.cos(longitude);
        final double sinLong = Math.sin(longitude);

        result.setElementAtIndex(0, -sinLat * cosLong);
        result.setElementAtIndex(1, -sinLong);
        result.setElementAtIndex(2, -cosLat * cosLong);

        result.setElementAtIndex(3, -sinLat * sinLong);
        result.setElementAtIndex(4, cosLong);
        result.setElementAtIndex(5, -cosLat * sinLong);

        result.setElementAtIndex(6, cosLat);
        result.setElementAtIndex(7, 0.0);
        result.setElementAtIndex(8, -sinLat);
    }

    /**
     * Computes matrix to convert ECEF to NED coordinates.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return a new matrix to convert ECEF to NED coordinates.
     */
    public static Matrix ecefToNedMatrix(final double latitude, final double longitude) {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformationMatrix.ROWS, CoordinateTransformationMatrix.COLS);
            ecefToNedMatrix(latitude, longitude, result);
        } catch (WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes ECEF to NED coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where result will be stored.
     */
    public static void ecefToNedCoordinateTransformationMatrix(final double latitude, final double longitude,
                                                               final CoordinateTransformationMatrix result) {
        try {
            result.setSourceType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setDestinationType(FrameType.LOCAL_NAVIGATION_FRAME);
            result.setMatrix(ecefToNedMatrix(latitude, longitude));
        } catch (InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Computes ECEF to NED coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return a new ECEF to NED coordinate transformation matrix.
     */
    public static CoordinateTransformationMatrix ecefToNedCoordinateTransformationMatrix(final double latitude,
                                                                                         final double longitude) {
        final CoordinateTransformationMatrix result = new CoordinateTransformationMatrix(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        ecefToNedCoordinateTransformationMatrix(latitude, longitude, result);
        return result;
    }

    /**
     * Computes matrix to convert NED to ECEF coordinates.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where computed matrix will be stored.
     */
    public static void nedToEcefMatrix(final double latitude, final double longitude, final Matrix result) {
        //NED to ECEF matrix is the inverse of ECEF to NED matrix.
        //Since ECEF to NED matrix is a rotation (3x3 and orthonormal), its inverse is the transpose.
        ecefToNedMatrix(latitude, longitude, result);
        result.transpose();
    }

    /**
     * Computes matrix to convert NED to ECEF coordinates.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return a new matrix to convert NED to ECEF coordinates.
     */
    public static Matrix nedToEcefMatrix(final double latitude, final double longitude) {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformationMatrix.ROWS, CoordinateTransformationMatrix.COLS);
            nedToEcefMatrix(latitude, longitude, result);
        } catch (WrongSizeException ignore) {
            // never happens
            result = null;
        }

        return result;
    }

    /**
     * Computes NED to ECEF coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where result will be stored.
     */
    public static void nedToEcefCoordinateTransformationMatrix(final double latitude, final double longitude,
                                                               final CoordinateTransformationMatrix result) {
        try {
            result.setSourceType(FrameType.LOCAL_NAVIGATION_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(nedToEcefMatrix(latitude, longitude));
        } catch (InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Computes NED to ECEF coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return instance where computed matrix will be stored.
     */
    public static CoordinateTransformationMatrix nedToEcefCoordinateTransformationMatrix(final double latitude,
                                                                                         final double longitude) {
        final CoordinateTransformationMatrix result = new CoordinateTransformationMatrix(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        nedToEcefCoordinateTransformationMatrix(latitude, longitude, result);
        return result;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new CoordinateTransformationMatrix(this);
    }
}
