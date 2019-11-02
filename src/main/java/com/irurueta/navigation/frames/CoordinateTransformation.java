package com.irurueta.navigation.frames;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a coordinate transformation matrix, or rotation matrix. The coordinate transformation matrix is a 3x3
 * matrix where a vector may be transformed in one step from one set of resolving axes to another by pre-multiplying it
 * by the appropriate coordinate transformation matrix. The coordinate transformation matrix is associated to a source
 * and destination frame.
 */
@SuppressWarnings("WeakerAccess")
public class CoordinateTransformation implements Serializable, Cloneable {

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
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * 3x3 matrix containing a rotation.
     */
    Matrix mMatrix;

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
    public CoordinateTransformation(final FrameType sourceType, final FrameType destinationType) {
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
    public CoordinateTransformation(final Matrix matrix, final FrameType sourceType,
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
    public CoordinateTransformation(final Matrix matrix, final FrameType sourceType,
                                    final FrameType destinationType) throws InvalidRotationMatrixException {
        this(matrix, sourceType, destinationType, DEFAULT_THRESHOLD);
    }

    /**
     * Constructor with Euler angles.
     * Notice that these angles do not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param roll            roll Euler angle (around x-axis) expressed in radians.
     * @param pitch           pitch Euler angle (around y-axis) expressed in radians.
     * @param yaw             yaw Euler angle (around z-axis) expressed in radians.
     * @param sourceType      source frame type.
     * @param destinationType destination frame type.
     */
    public CoordinateTransformation(final double roll, final double pitch, final double yaw,
                                    final FrameType sourceType, final FrameType destinationType) {
        this(sourceType, destinationType);
        setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Constructor with Euler angles.
     * Notice that these angles do not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param roll            roll Euler angle (around x-axis).
     * @param pitch           pitch Euler angle (around y-axis).
     * @param yaw             yaw Euler angle (around z-axis).
     * @param sourceType      source frame type.
     * @param destinationType destination frame type.
     */
    public CoordinateTransformation(final Angle roll, final Angle pitch, final Angle yaw,
                                    final FrameType sourceType, final FrameType destinationType) {
        this(sourceType, destinationType);
        setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Constructor.
     *
     * @param input other coordinate transformation matrix to copy data from.
     */
    public CoordinateTransformation(final CoordinateTransformation input) {
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
     * Gets roll Euler angle (around x-axis) expressed in radians.
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return roll Euler angle.
     */
    public double getRollEulerAngle() {
        return Math.atan2(mMatrix.getElementAt(1, 2), mMatrix.getElementAt(2, 2));
    }

    /**
     * Gets roll Euler angle (around x-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param result instance where roll Euler angle will be stored.
     */
    public void getRollEulerAngleMeasurement(final Angle result) {
        result.setValue(getRollEulerAngle());
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets roll Euler angle (around x-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return roll Euler angle.
     */
    public Angle getRollEulerAngleMeasurement() {
        final Angle result = new Angle(0.0, AngleUnit.RADIANS);
        getRollEulerAngleMeasurement(result);
        return result;
    }

    /**
     * Gets pitch Euler angle (around y-axis) expressed in radians.
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return pitch Euler angle.
     */
    public double getPitchEulerAngle() {
        return -Math.asin(mMatrix.getElementAt(0, 2));
    }

    /**
     * Gets pitch Euler angle (around y-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param result instance where pitch Euler angle will be stored.
     */
    public void getPitchEulerAngleMeasurement(final Angle result) {
        result.setValue(getPitchEulerAngle());
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets pitch Euler angle (around y-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return pitch Euler angle.
     */
    public Angle getPitchEulerAngleMeasurement() {
        final Angle result = new Angle(0.0, AngleUnit.RADIANS);
        getPitchEulerAngleMeasurement(result);
        return result;
    }

    /**
     * Gets yaw Euler angle (around z-axis) expressed in radians.
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return yaw Euler angle.
     */
    public double getYawEulerAngle() {
        return Math.atan2(mMatrix.getElementAt(0, 1), mMatrix.getElementAt(0, 0));
    }

    /**
     * Gets yaw Euler angle (around z-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param result instance where yaw Euler angle will be stored.
     */
    public void getYawEulerAngleMeasurement(final Angle result) {
        result.setValue(getYawEulerAngle());
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets yaw Euler angle (around z-axis).
     * Notice that this angle does not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @return yaw Euler angle.
     */
    public Angle getYawEulerAngleMeasurement() {
        final Angle result = new Angle(0.0, AngleUnit.RADIANS);
        getYawEulerAngleMeasurement(result);
        return result;
    }

    /**
     * Sets euler angles (roll, pitch and yaw) expressed in radians.
     * Notice that these angles do not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param roll  roll Euler angle (around x-axis) expressed in radians.
     * @param pitch pitch Euler angle (around y-axis) expressed in radians.
     * @param yaw   yaw Euler angle (around z-axis) expressed in radians.
     */
    public void setEulerAngles(final double roll, final double pitch, final double yaw) {
        final double sinPhi = Math.sin(roll);
        final double cosPhi = Math.cos(roll);
        final double sinTheta = Math.sin(pitch);
        final double cosTheta = Math.cos(pitch);
        final double sinPsi = Math.sin(yaw);
        final double cosPsi = Math.cos(yaw);

        // Calculate coordinate transformation matrix using (2.22)
        mMatrix.setElementAt(0, 0, cosTheta * cosPsi);
        mMatrix.setElementAt(0, 1, cosTheta * sinPsi);
        mMatrix.setElementAt(0, 2, -sinTheta);

        mMatrix.setElementAt(1, 0, -cosPhi * sinPsi + sinPhi * sinTheta * cosPsi);
        mMatrix.setElementAt(1, 1, cosPhi * cosPsi + sinPhi * sinTheta * sinPsi);
        mMatrix.setElementAt(1, 2, sinPhi * cosTheta);

        mMatrix.setElementAt(2, 0, sinPhi * sinPsi + cosPhi * sinTheta * cosPsi);
        mMatrix.setElementAt(2, 1, -sinPhi * cosPsi + cosPhi * sinTheta * sinPsi);
        mMatrix.setElementAt(2, 2, cosPhi * cosTheta);
    }

    /**
     * Sets euler angles (roll, pitch and yaw).
     * Notice that these angles do not match angles obtained from {@link com.irurueta.geometry.Rotation3D} or
     * {@link com.irurueta.geometry.Quaternion} because they are referred to different axes.
     *
     * @param roll  roll Euler angle (around x-axis).
     * @param pitch pitch Euler angle (around y-axis).
     * @param yaw   yaw Euler angle (around z-axis).
     */
    public void setEulerAngles(final Angle roll, final Angle pitch, final Angle yaw) {
        setEulerAngles(AngleConverter.convert(roll.getValue().doubleValue(), roll.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(pitch.getValue().doubleValue(), pitch.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(yaw.getValue().doubleValue(), yaw.getUnit(), AngleUnit.RADIANS));
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
    public void copyTo(final CoordinateTransformation output) {
        output.mSourceType = mSourceType;
        output.mDestinationType = mDestinationType;
        mMatrix.copyTo(output.mMatrix);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final CoordinateTransformation input) {
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
        if (!(obj instanceof CoordinateTransformation)) {
            return false;
        }

        final CoordinateTransformation other = (CoordinateTransformation) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final CoordinateTransformation other) {
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
    public boolean equals(final CoordinateTransformation other, final double threshold) {
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
    public void inverse(final CoordinateTransformation result) {
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
    public CoordinateTransformation inverseAndReturnNew() {
        final CoordinateTransformation result = new CoordinateTransformation(mDestinationType, mSourceType);
        inverse(result);
        return result;
    }

    /**
     * Computes matrix to convert ECEF to NED coordinates.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where computed matrix will be stored.
     */
    public static void ecefToNedMatrix(final Angle latitude, final Angle longitude, final Matrix result) {
        ecefToNedMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS),
                result);
    }

    /**
     * Computes matrix to convert ECEF to NED coordinates.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return a new matrix to convert ECEF to NED coordinates.
     */
    public static Matrix ecefToNedMatrix(final Angle latitude, final Angle longitude) {
        return ecefToNedMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS));
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
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
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
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where result will be stored.
     */
    public static void ecefToNedCoordinateTransformationMatrix(final Angle latitude, final Angle longitude,
                                                               final CoordinateTransformation result) {
        ecefToNedCoordinateTransformationMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS),
                result);
    }

    /**
     * Computes ECEF to NED coordinate transformation matrix.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return a new ECEF to NED coordinate transformation matrix.
     */
    public static CoordinateTransformation ecefToNedCoordinateTransformationMatrix(
            final Angle latitude, final Angle longitude) {
        return ecefToNedCoordinateTransformationMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS));
    }

    /**
     * Computes ECEF to NED coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where result will be stored.
     */
    public static void ecefToNedCoordinateTransformationMatrix(final double latitude, final double longitude,
                                                               final CoordinateTransformation result) {
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
    public static CoordinateTransformation ecefToNedCoordinateTransformationMatrix(final double latitude,
                                                                                   final double longitude) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        ecefToNedCoordinateTransformationMatrix(latitude, longitude, result);
        return result;
    }

    /**
     * Computes matrix to convert NED to ECEF coordinates.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where computed matrix will be stored.
     */
    public static void nedToEcefMatrix(final Angle latitude, final Angle longitude, final Matrix result) {
        nedToEcefMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS),
                result);
    }

    /**
     * Computes matrix to convert NED to ECEF coordinates.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return a new matrix to convert NED to ECEF coordinates.
     */
    public static Matrix nedToEcefMatrix(final Angle latitude, final Angle longitude) {
        return nedToEcefMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS));
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
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
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
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where result will be stored.
     */
    public static void nedToEcefCoordinateTransformationMatrix(final Angle latitude, final Angle longitude,
                                                               final CoordinateTransformation result) {
        nedToEcefCoordinateTransformationMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS),
                result);
    }

    /**
     * Computes NED to ECEF coordinate transformation matrix.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return a new NED to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation nedToEcefCoordinateTransformationMatrix(
            final Angle latitude, final Angle longitude) {
        return nedToEcefCoordinateTransformationMatrix(
                AngleConverter.convert(latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                AngleConverter.convert(longitude.getValue().doubleValue(), longitude.getUnit(), AngleUnit.RADIANS));
    }

    /**
     * Computes NED to ECEF coordinate transformation matrix.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where result will be stored.
     */
    public static void nedToEcefCoordinateTransformationMatrix(final double latitude, final double longitude,
                                                               final CoordinateTransformation result) {
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
     * @return a new NED to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation nedToEcefCoordinateTransformationMatrix(final double latitude,
                                                                                   final double longitude) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        nedToEcefCoordinateTransformationMatrix(latitude, longitude, result);
        return result;
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @param result       instance where result will be stored.
     */
    public static void ecefToEciMatrixFromTimeInterval(final Time timeInterval, final Matrix result) {
        ecefToEciMatrixFromTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND), result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param result       instance where result will be stored.
     */
    public static void ecefToEciMatrixFromTimeInterval(final double timeInterval, final Matrix result) {
        ecefToEciMatrixFromAngle(EARTH_ROTATION_RATE * timeInterval, result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated.
     * @param result instance where result will be stored.
     */
    public static void ecefToEciMatrixFromAngle(final Angle angle, final Matrix result) {
        ecefToEciMatrixFromAngle(AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS), result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated expressed in radians.
     * @param result instance where result will be stored.
     */
    public static void ecefToEciMatrixFromAngle(final double angle, final Matrix result) {
        if (result.getRows() != ROWS || result.getColumns() != COLS) {
            try {
                result.resize(ROWS, COLS);
            } catch (final WrongSizeException ignore) {
            }
        }

        final double sinAngle = Math.sin(angle);
        final double cosAngle = Math.cos(angle);

        result.setElementAt(0, 0, cosAngle);
        result.setElementAt(1, 0, sinAngle);
        result.setElementAt(2, 0, 0.0);

        result.setElementAt(0, 1, -sinAngle);
        result.setElementAt(1, 1, cosAngle);
        result.setElementAt(2, 1, 0.0);

        result.setElementAt(0, 2, 0.0);
        result.setElementAt(1, 2, 0.0);
        result.setElementAt(2, 2, 1.0);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static Matrix ecefToEciMatrixFromTimeInterval(final Time timeInterval) {
        return ecefToEciMatrixFromTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static Matrix ecefToEciMatrixFromTimeInterval(final double timeInterval) {
        return ecefToEciMatrixFromAngle(EARTH_ROTATION_RATE * timeInterval);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static Matrix ecefToEciMatrixFromAngle(final Angle angle) {
        return ecefToEciMatrixFromAngle(AngleConverter.convert(
                angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS));
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated expressed in radians.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static Matrix ecefToEciMatrixFromAngle(final double angle) {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
            ecefToEciMatrixFromAngle(angle, result);
        } catch (WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @param result       instance where result will be stored.
     */
    public static void ecefToEciCoordinateTransformationMatrixFromTimeInterval(
            final Time timeInterval, final CoordinateTransformation result) {
        ecefToEciCoordinateTransformationMatrixFromTimeInterval(
                TimeConverter.convert(timeInterval.getValue().doubleValue(),
                        timeInterval.getUnit(), TimeUnit.SECOND), result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param result       instance where result will be stored.
     */
    public static void ecefToEciCoordinateTransformationMatrixFromTimeInterval(
            final double timeInterval, final CoordinateTransformation result) {
        ecefToEciCoordinateTransformationMatrixFromAngle(
                EARTH_ROTATION_RATE * timeInterval, result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated.
     * @param result instance where result will be stored.
     */
    public static void ecefToEciCoordinateTransformationMatrixFromAngle(
            final Angle angle, final CoordinateTransformation result) {
        ecefToEciCoordinateTransformationMatrixFromAngle(
                AngleConverter.convert(angle.getValue().doubleValue(),
                        angle.getUnit(), AngleUnit.RADIANS), result);
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated expressed in radians.
     * @param result instance where result will be stored.
     */
    public static void ecefToEciCoordinateTransformationMatrixFromAngle(
            final double angle, final CoordinateTransformation result) {
        try {
            result.setSourceType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_INERTIAL_FRAME);
            result.setMatrix(ecefToEciMatrixFromAngle(angle));
        } catch (InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static CoordinateTransformation ecefToEciCoordinateTransformationMatrixFromTimeInterval(
            final Time timeInterval) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        ecefToEciCoordinateTransformationMatrixFromTimeInterval(timeInterval, result);
        return result;
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static CoordinateTransformation ecefToEciCoordinateTransformationMatrixFromTimeInterval(
            final double timeInterval) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        ecefToEciCoordinateTransformationMatrixFromTimeInterval(timeInterval, result);
        return result;
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth rotation angle.
     *
     * @param angle angle amount the Earth has rotated.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static CoordinateTransformation ecefToEciCoordinateTransformationMatrixFromAngle(
            final Angle angle) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        ecefToEciCoordinateTransformationMatrixFromAngle(angle, result);
        return result;
    }

    /**
     * Computes ECEF to ECI coordinate transformation matrix for provided Earth rotation angle.
     *
     * @param angle angle amount the Earth has rotated expressed in radians.
     * @return a new ECEF to ECI coordinate transformation matrix.
     */
    public static CoordinateTransformation ecefToEciCoordinateTransformationMatrixFromAngle(
            final double angle) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        ecefToEciCoordinateTransformationMatrixFromAngle(angle, result);
        return result;
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @param result       instance where result will be stored.
     */
    public static void eciToEcefMatrixFromTimeInterval(final Time timeInterval,
                                                       final Matrix result) {
        eciToEcefMatrixFromTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND), result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param result       instance where result will be stored.
     */
    public static void eciToEcefMatrixFromTimeInterval(final double timeInterval,
                                                       final Matrix result) {
        eciToEcefMatrixFromAngle(EARTH_ROTATION_RATE * timeInterval, result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated.
     * @param result instance where result will be stored.
     */
    public static void eciToEcefMatrixFromAngle(final Angle angle, final Matrix result) {
        eciToEcefMatrixFromAngle(AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS), result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated expressed in radians.
     * @param result instance where result will be stored.
     */
    public static void eciToEcefMatrixFromAngle(final double angle, final Matrix result) {
        ecefToEciMatrixFromAngle(angle, result);
        result.transpose();
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static Matrix eciToEcefMatrixFromTimeInterval(final Time timeInterval) {
        return eciToEcefMatrixFromTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static Matrix eciToEcefMatrixFromTimeInterval(final double timeInterval) {
        return eciToEcefMatrixFromAngle(EARTH_ROTATION_RATE * timeInterval);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static Matrix eciToEcefMatrixFromAngle(final Angle angle) {
        return eciToEcefMatrixFromAngle(AngleConverter.convert(
                angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS));
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static Matrix eciToEcefMatrixFromAngle(final double angle) {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
            eciToEcefMatrixFromAngle(angle, result);
        } catch (WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @param result       instance where result will be stored.
     */
    public static void eciToEcefCoordinateTransformationMatrixFromTimeInterval(
            final Time timeInterval, final CoordinateTransformation result) {
        eciToEcefCoordinateTransformationMatrixFromTimeInterval(
                TimeConverter.convert(timeInterval.getValue().doubleValue(),
                        timeInterval.getUnit(), TimeUnit.SECOND), result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param result       instance where result will be stored.
     */
    public static void eciToEcefCoordinateTransformationMatrixFromTimeInterval(
            final double timeInterval, final CoordinateTransformation result) {
        eciToEcefCoordinateTransformationMatrixFromAngle(
                EARTH_ROTATION_RATE * timeInterval, result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated.
     * @param result instance where result will be stored.
     */
    public static void eciToEcefCoordinateTransformationMatrixFromAngle(
            final Angle angle, final CoordinateTransformation result) {
        eciToEcefCoordinateTransformationMatrixFromAngle(
                AngleConverter.convert(angle.getValue().doubleValue(),
                        angle.getUnit(), AngleUnit.RADIANS), result);
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle  angle amount the Earth has rotated expressed in radians.
     * @param result instance where result will be stored.
     */
    public static void eciToEcefCoordinateTransformationMatrixFromAngle(
            final double angle, final CoordinateTransformation result) {
        try {
            result.setSourceType(FrameType.EARTH_CENTERED_INERTIAL_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(eciToEcefMatrixFromAngle(angle));
        } catch (InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation eciToEcefCoordinateTransformationMatrixFromTimeInterval(
            final Time timeInterval) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_INERTIAL_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        eciToEcefCoordinateTransformationMatrixFromTimeInterval(timeInterval, result);
        return result;
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix taking into account Earth
     * rotation during provided time interval.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation eciToEcefCoordinateTransformationMatrixFromInterval(
            final double timeInterval) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_INERTIAL_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        eciToEcefCoordinateTransformationMatrixFromTimeInterval(timeInterval, result);
        return result;
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation eciToEcefCoordinateTransformationMatrixFromAngle(
            final Angle angle) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_INERTIAL_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        eciToEcefCoordinateTransformationMatrixFromAngle(angle, result);
        return result;
    }

    /**
     * Computes ECI to ECEF coordinate transformation matrix for provided Earth
     * rotation angle.
     *
     * @param angle angle amount the Earth has rotated expressed in radians.
     * @return a new ECI to ECEF coordinate transformation matrix.
     */
    public static CoordinateTransformation eciToEcefCoordinateTransformationMatrixFromAngle(
            final double angle) {
        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.EARTH_CENTERED_INERTIAL_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        eciToEcefCoordinateTransformationMatrixFromAngle(angle, result);
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
        return new CoordinateTransformation(this);
    }
}
