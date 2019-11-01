package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;

/**
 * Base class for ECI or ECEF frames containing common logic and data for such frames.
 */
@SuppressWarnings("WeakerAccess")
public abstract class ECIorECEFFrame<T extends ECIorECEFFrame> implements Frame, Serializable, Cloneable {

    /**
     * Number of coordinates representing position.
     */
    public static final int NUM_POSITION_COORDINATES = 3;

    /**
     * Number of coordinates representing velocity.
     */
    public static final int NUM_VELOCITY_COORDINATES = 3;

    /**
     * Cartesian x coordinate of body position expressed in meters (m) with respect ECI or ECEF frame, resolved along
     * the corresponding frame axes.
     */
    double mX;

    /**
     * Catesian y coordinate of body position expressed in meters (m) with respect ECI or ECEF frame, resolved along
     * the corresponding frame axes.
     */
    double mY;

    /**
     * Cartesian z coordinate of body position expressed in meters (m) with respect ECI or ECEF frame, resolved along
     * the corresponding frame axes.
     */
    double mZ;

    /**
     * X coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECI or ECEF frame,
     * resolved along the corresponding frame axes.
     */
    double mVx;

    /**
     * Y coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECI or ECEF frame,
     * resolved along the corresponding frame axes.
     */
    double mVy;

    /**
     * Z coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECI or ECEF frame,
     * resolved along the corresponding frame axes.
     */
    double mVz;

    /**
     * Body to ECI frame coordinate transformation matrix.
     */
    CoordinateTransformation mC;

    /**
     * Constructor.
     */
    ECIorECEFFrame() { }

    /**
     * Constructor.
     *
     * @param c Body to ECI or ECEF coordinate transformation.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    ECIorECEFFrame(final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        setCoordinateTransformation(c);
    }

    /**
     * Gets cartesian x coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @return cartesian x coordinate of body position.
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets cartesian x coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position.
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets cartesian y coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @return cartesian y coordinate of body position.
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets cartesian y coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param y cartesian y coordinate of body position.
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets cartesian z coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @return cartesian z coordinate of body position.
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets cartesian z coordinate of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param z cartesian z coordinate of body position.
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets cartesian coordinates of body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position, resolved along ECI or ECEF-frame axes.
     * @param y cartesian y coordinate of body position, resolved along ECI or ECEF-frame axes.
     * @param z cartesian z coordinate of body position, resolved along ECI or ECEF-frame axes.
     */
    public void setCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @return body position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where position data is copied to.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Sets body position expressed in meters (m) and resolved along ECI or ECEF-frame axes.
     *
     * @param point body position to be set.
     */
    public void setPosition(final Point3D point) {
        mX = point.getInhomX();
        mY = point.getInhomY();
        mZ = point.getInhomZ();
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where cartesian x coordinate of body position will be stored.
     */
    public void getPositionX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @return x coordinate of body position resolved along ECI or ECEF-frame axes.
     */
    public Distance getPositionX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     */
    public void setPositionX(final Distance positionX) {
        mX = DistanceConverter.convert(positionX.getValue().doubleValue(),
                positionX.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where cartesian y coordinate of body position will be stored.
     */
    public void getPositionY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @return y coordinate of body position resolved along ECI or ECEF-frame axes.
     */
    public Distance getPositionY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param positionY cartesian y coordinate of body position to be set.
     */
    public void setPositionY(final Distance positionY) {
        mY = DistanceConverter.convert(positionY.getValue().doubleValue(),
                positionY.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where cartesian z coordinate of body position will be stored.
     */
    public void getPositionZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @return z coordinate of body position resolved along ECI or ECEF-frame axes.
     */
    public Distance getPositionZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of body position resolved along ECI or ECEF-frame axes.
     *
     * @param positionZ cartesian z coordinate of body position to be set.
     */
    public void setPositionZ(final Distance positionZ) {
        mZ = DistanceConverter.convert(positionZ.getValue().doubleValue(),
                positionZ.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets cartesian coordinates of body position resolved along ECI or ECEF-frame axes.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECI or ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECI or ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECI or ECEF-frame axes.
     */
    public void setPositionCoordinates(final Distance positionX,
                                       final Distance positionY,
                                       final Distance positionZ) {
        setPositionX(positionX);
        setPositionY(positionY);
        setPositionZ(positionZ);
    }

    /**
     * Gets norm of position expressed in meters (m), which represents the distance to
     * Earth's center of mass.
     *
     * @return position norm expressed in meters (m).
     */
    public double getPositionNorm() {
        return Math.sqrt(mX * mX + mY * mY + mZ * mZ);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionNormAsDistance(final Distance result) {
        result.setValue(getPositionNorm());
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @return position norm.
     */
    public Distance getPositionNormAsDistance() {
        return new Distance(getPositionNorm(), DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @return x coordinate of velocity.
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @return y coordinate of velocity.
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @param vy y coordinate of velocity.
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @return z coordinate of velocity.
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @param vz z coordinate of velocity.
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s) resolved along ECI or
     * ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     * @param vy y coordinate of velocity.
     * @param vz z coordinate of velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getVelocityNorm() {
        return Math.sqrt(mVx * mVx + mVy * mVy + mVz * mVz);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @param result velocity norm.
     */
    public void getVelocityNormAsSpeed(final Speed result) {
        result.setValue(getVelocityNorm());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @return velocity norm.
     */
    public Speed getVelocityNormAsSpeed() {
        return new Speed(getVelocityNorm(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param result instance where x coordinate of velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @return x coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param speedX x coordinate of velocity of body frame resolved along ECI or ECEF-frame
     *               axes to be set.
     */
    public void setSpeedX(final Speed speedX) {
        mVx = SpeedConverter.convert(speedX.getValue().doubleValue(),
                speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where y coordinate of velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @return y coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param speedY y coordinate of velocity of body frame resolved along ECI or ECEF-frame
     *               axes to be set.
     */
    public void setSpeedY(final Speed speedY) {
        mVy = SpeedConverter.convert(speedY.getValue().doubleValue(),
                speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param result instance where z coordinate of velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @return z coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of velocity of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param speedZ z coordinate of velocity of body frame resolved along ECI or ECEF-frame
     *               axes to be set.
     */
    public void setSpeedZ(final Speed speedZ) {
        mVz = SpeedConverter.convert(speedZ.getValue().doubleValue(),
                speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity coordinates of body frame resolved along ECI or ECEF-frame axes.
     *
     * @param speedX x coordinate of velocity to be set.
     * @param speedY y coordinate of velocity to be set.
     * @param speedZ z coordinate of velocity to be set.
     */
    public void setSpeedCoordinates(final Speed speedX,
                                    final Speed speedY,
                                    final Speed speedZ) {
        setSpeedX(speedX);
        setSpeedY(speedY);
        setSpeedZ(speedZ);
    }

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation(), but more efficient
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public Matrix getCoordinateTransformationMatrix() {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
            getCoordinateTransformationMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    @Override
    public void getCoordinateTransformationMatrix(final Matrix result) {
        mC.mMatrix.copyTo(result);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final T output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;

        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;

        mC.copyTo(output.mC);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final T input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;

        mC.copyFrom(input.mC);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final T other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position, velocity and coordinate transformation matrix.
     * @return true if both instances are considered to be equal (up to provided threshold), false otherwise.
     */
    public boolean equals(final T other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold
                && mC.equals(other.mC, threshold);
    }
}
