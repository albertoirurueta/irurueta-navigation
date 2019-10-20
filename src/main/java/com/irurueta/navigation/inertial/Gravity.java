package com.irurueta.navigation.inertial;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains acceleration due to gravity resolved about ECEF frame.
 */
@SuppressWarnings("WeakerAccess")
public class Gravity implements Serializable, Cloneable {

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
    public boolean equals(Object obj) {
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
