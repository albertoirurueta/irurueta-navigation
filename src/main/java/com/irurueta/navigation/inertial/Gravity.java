package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains acceleration due to gravity resolved about ECEF frame.
 */
@SuppressWarnings("WeakerAccess")
public class Gravity implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * Acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     */
    private double mGx;

    /**
     * Acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     */
    private double mGy;

    /**
     * Acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    private double mGz;

    /**
     * Constructor.
     */
    public Gravity() { }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public Gravity(final double gx, final double gy, final double gz) {
        setCoordinates(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis to be set.
     * @param gy acceleration due to gravity through ECEF y-axis to be set.
     * @param gz acceleration due to gravity through ECEF z-axis to be set.
     */
    public Gravity(final Acceleration gx, final Acceleration gy,
                   final Acceleration gz) {
        setGravityCoordinates(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public Gravity(final Gravity input) {
        copyFrom(input);
    }

    /**
     * Gets acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECEF x-axis.
     */
    public double getGx() {
        return mGx;
    }

    /**
     * Sets acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     *
     * @param gx acceleration due to gravity through ECEF x-axis.
     */
    public void setGx(final double gx) {
        mGx = gx;
    }

    /**
     * Gets acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECEF y-axis.
     */
    public double getGy() {
        return mGy;
    }

    /**
     * Sets acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     *
     * @param gy acceleration due to gravity through ECEF y-axis.
     */
    public void setGy(final double gy) {
        mGy = gy;
    }

    /**
     * Gets acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECEF z-axis.
     */
    public double getGz() {
        return mGz;
    }

    /**
     * Sets acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     *
     * @param gz acceleration due to gravity through ECEF z-axis.
     */
    public void setGz(final double gz) {
        mGz = gz;
    }

    /**
     * Sets gravity coordinates resolved about ECEF frame and expressed in meters per squared second (m/s^2).
     *
     * @param gx acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public void setCoordinates(final double gx, final double gy, final double gz) {
        mGx = gx;
        mGy = gy;
        mGz = gz;
    }

    /**
     * Gets acceleration due to gravity through ECEF x-axis.
     *
     * @param result instance where acceleration due to gravity through ECEF x-axis will be stored.
     */
    public void getGravityX(final Acceleration result) {
        result.setValue(mGx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECEF x-axis.
     *
     * @return acceleration due to gravity through ECEF x-axis.
     */
    public Acceleration getGravityX() {
        return new Acceleration(mGx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECEF x-axis.
     *
     * @param gravityX acceleration due to gravity through ECEF x-axis to be set.
     */
    public void setGravityX(final Acceleration gravityX) {
        mGx = AccelerationConverter.convert(gravityX.getValue().doubleValue(),
                gravityX.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECEF y-axis.
     *
     * @param result instance where acceleration due to gravity through ECEF y-axis will be stored.
     */
    public void getGravityY(final Acceleration result) {
        result.setValue(mGy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECEF y-axis.
     *
     * @return acceleration due to gravity through ECEF y-axis.
     */
    public Acceleration getGravityY() {
        return new Acceleration(mGy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECEF y-axis.
     *
     * @param gravityY acceleration due to gravity through ECEF y-axis to be set.
     */
    public void setGravityY(final Acceleration gravityY) {
        mGy = AccelerationConverter.convert(gravityY.getValue().doubleValue(),
                gravityY.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECEF z-axis.
     *
     * @param result instance where acceleration due to gravity through ECEF z-axis will be stored.
     */
    public void getGravityZ(final Acceleration result) {
        result.setValue(mGz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECEF z-axis.
     *
     * @return acceleration due to gravity through ECEF z-axis.
     */
    public Acceleration getGravityZ() {
        return new Acceleration(mGz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECEF z-axis.
     *
     * @param gravityZ acceleration due to gravity through ECEF z-axis to be set.
     */
    public void setGravityZ(final Acceleration gravityZ) {
        mGz = AccelerationConverter.convert(gravityZ.getValue().doubleValue(),
                gravityZ.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets gravity coordinates.
     *
     * @param gravityX acceleration due to gravity through ECEF x-axis to be set.
     * @param gravityY acceleration due to gravity through ECEF y-axis to be set.
     * @param gravityZ acceleration due to gravity through ECEF z-axis to be set.
     */
    public void setGravityCoordinates(final Acceleration gravityX,
                                      final Acceleration gravityY,
                                      final Acceleration gravityZ) {
        setGravityX(gravityX);
        setGravityY(gravityY);
        setGravityZ(gravityZ);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final Gravity output) {
        output.mGx = mGx;
        output.mGy = mGy;
        output.mGz = mGz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final Gravity input) {
        mGx = input.mGx;
        mGy = input.mGy;
        mGz = input.mGz;
    }

    /**
     * Gets gravity coordinates as an array.
     *
     * @param result array instance where gravity coordinates will be stored in
     *               x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mGx;
        result[1] = mGy;
        result[2] = mGz;
    }

    /**
     * Gets gravity coordinates as an array.
     *
     * @return array containing gravity coordinates in x,y,z order.
     */
    public double[] asArray() {
        final double[] result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets gravity coordinates as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where gravity coordinates will be stored in
     *               x,y,z order.
     */
    public void asMatrix(final Matrix result) {
        if (result.getColumns() != COMPONENTS || result.getRows() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mGx);
        result.setElementAtIndex(1, mGy);
        result.setElementAtIndex(2, mGz);
    }

    /**
     * Gets gravity coordinates as a column matrix.
     *
     * @return a matrix containing gravity coordinates stored in x,y,z order.
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
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mGx, mGy, mGz);
    }

    /**
     * Checks if provided object is a Gravity instance having exactly the same contents
     * as this instance.
     *
     * @param obj object to be compared.
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
        if (!(obj instanceof Gravity)) {
            return false;
        }

        final Gravity other = (Gravity) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final Gravity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other instance to be compared.
     * @param threshold maximum allowed difference between gravity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final Gravity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mGx - other.mGx) <= threshold
                && Math.abs(mGy - other.mGy) <= threshold
                && Math.abs(mGz - other.mGz) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"CloneDoesntDeclareCloneNotSupportedException", "MethodDoesntCallSuperMethod"})
    @Override
    protected Object clone() {
        return new Gravity(this);
    }
}
