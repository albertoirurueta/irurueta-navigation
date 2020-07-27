package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body velocity with respect Earth, resolved about ECEF frame
 * and expressed in meters per second (m/s).
 */
@SuppressWarnings("WeakerAccess")
public class ECEFVelocity implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * X coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double mVx;

    /**
     * Y coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double mVy;

    /**
     * Z coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double mVz;

    /**
     * Constructor.
     */
    public ECEFVelocity() {
    }

    /**
     * Constructor.
     *
     * @param vx x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vy y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vz z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     */
    public ECEFVelocity(final double vx, final double vy, final double vz) {
        setCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param vx x coordinate of velocity of body frame and resolved along ECEF-frame
     *           axes.
     * @param vy y coordinate of velocity of body frame and resolved along ECEF-frame
     *           axes.
     * @param vz z coordinate of velocity of body frame and resolved along ECEF-frame
     *           axes.
     */
    public ECEFVelocity(final Speed vx, final Speed vy, final Speed vz) {
        setCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param input Body velocity to copy data from.
     */
    public ECEFVelocity(final ECEFVelocity input) {
        copyFrom(input);
    }

    /**
     * Gets x coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @return x coordinate of velocity.
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity.
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vy y coordinate of velocity.
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity.
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vz z coordinate of velocity.
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     * @param vy y coordinate of velocity.
     * @param vz z coordinate of velocity.
     */
    public void setCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets cartesian x coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian x coordinate of velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return x coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of body velocity to be set.
     */
    public void setVx(final Speed vx) {
        mVx = convertSpeed(vx);
    }

    /**
     * Gets cartesian y coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian y coordinate of velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vy y coordinate of body velocity to be set.
     */
    public void setVy(final Speed vy) {
        mVy = convertSpeed(vy);
    }

    /**
     * Gets cartesian z coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian z coordinate of velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vz z coordinate of body velocity to be set.
     */
    public void setVz(final Speed vz) {
        mVz = convertSpeed(vz);
    }

    /**
     * Sets cartesian coordinates of body velocity resolved along ECEF-frame axes.
     *
     * @param vx x cartesian coordinate of body velocity.
     * @param vy y cartesian coordinate of body velocity.
     * @param vz z cartesian coordinate of body velocity.
     */
    public void setCoordinates(final Speed vx, final Speed vy, final Speed vz) {
        setVx(vx);
        setVy(vy);
        setVz(vz);
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getNorm() {
        return Math.sqrt(mVx * mVx + mVy * mVy + mVz * mVz);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @param result velocity norm.
     */
    public void getNormAsSpeed(final Speed result) {
        result.setValue(getNorm());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @return velocity norm.
     */
    public Speed getNormAsSpeed() {
        return new Speed(getNorm(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final ECEFVelocity output) {
        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFVelocity input) {
        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;
    }

    /**
     * Gets velocity coordinates expressed in meters per second (m/s)
     * as an array.
     *
     * @param result array instance where velocity coordinates will
     *               be stored in x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mVx;
        result[1] = mVy;
        result[2] = mVz;
    }

    /**
     * Gets velocity coordinates expressed in meters per second (m/s) as an array.
     *
     * @return array containing velocity coordinates in x,y,z order.
     */
    public double[] asArray() {
        final double[] result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets velocity coordinates expressed in meters per second (m/s) as a column
     * matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where velocity coordinates will be stored
     *               in x,y,z order.
     */
    public void asMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mVx);
        result.setElementAtIndex(1, mVy);
        result.setElementAtIndex(2, mVz);
    }

    /**
     * Gets velocity coordinates expressed in meters per second (m/s) as a column
     * matrix.
     *
     * @return velocity coordinates stored in x,y,z order.
     */
    public Matrix asMatrix() {
        Matrix result;
        try {
            result = new Matrix(COMPONENTS, 1);
            asMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in
     * collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mVx, mVy, mVz);
    }

    /**
     * Checks if provided object is an ECEFVelocity having exactly the same contents
     * as this instance.
     *
     * @param o Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        final ECEFVelocity other = (ECEFVelocity) o;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final ECEFVelocity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between velocity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final ECEFVelocity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final ECEFVelocity result = (ECEFVelocity) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts speed instance into meters per second value.
     *
     * @param speed instance to be converted.
     * @return converted value.
     */
    private double convertSpeed(final Speed speed) {
        return SpeedConverter.convert(speed.getValue().doubleValue(),
                speed.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }
}
