package com.irurueta.navigation.inertial;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body cartesian position with respect Earth, resolved about ECEF frame
 * and expressed in meters (m).
 */
@SuppressWarnings("WeakerAccess")
public class ECEFPosition implements Serializable, Cloneable {

    /**
     * Cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double mX;

    /**
     * Cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double mY;

    /**
     * Cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double mZ;

    /**
     * Constructor.
     */
    public ECEFPosition() {
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) and
     *          resolved along ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) and
     *          resolved along ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) and
     *          resolved along ECEF-frame axes.
     */
    public ECEFPosition(final double x, final double y, final double z) {
        setCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position resolved along ECEF-frame axes.
     * @param y cartesian y coordinate of body position resolved along ECEF-frame axes.
     * @param z cartesian z coordinate of body position resolved along ECEF-frame axes.
     */
    public ECEFPosition(final Distance x, final Distance y, final Distance z) {
        setCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param input ECEF body position to copy data from.
     */
    public ECEFPosition(final ECEFPosition input) {
        copyFrom(input);
    }

    /**
     * Gets cartesian x coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @return cartesian x coordinate of body position.
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets cartesian x coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param x cartesian x coordinate of body position.
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets cartesian y coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @return cartesian y coordinate of body position.
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets cartesian y coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param y cartesian y coordinate of body position.
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets cartesian z coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @return z coordinate of body position.
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets cartesian z coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param z cartesian z coordinate of body position.
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets cartesian coordinates of body position expressed in meters (m) and resolved
     * along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position.
     * @param y cartesian y coordinate of body position.
     * @param z cartesian z coordinate of body position.
     */
    public void setCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return body position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param result instance where position data is copied to.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Sets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param point body position to be set.
     */
    public void setPosition(final Point3D point) {
        mX = point.getInhomX();
        mY = point.getInhomY();
        mZ = point.getInhomZ();
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian x coordinate of body position will be
     *               stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @return x coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position to be set.
     */
    public void setX(final Distance x) {
        mX = convertDistance(x);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian y coordinate of body position will be
     *               stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @return y coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param y cartesian y coordinate of body position to be set.
     */
    public void setY(final Distance y) {
        mY = convertDistance(y);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian z coordinate of body position will be
     *               stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @return z coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param z cartesian z coordinate of body position to be set.
     */
    public void setZ(final Distance z) {
        mZ = convertDistance(z);
    }

    /**
     * Sets cartesian coordinates of body position and resolved along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position.
     * @param y cartesian y coordinate of body position.
     * @param z cartesian z coordinate of body position.
     */
    public void setCoordinates(final Distance x, final Distance y, final Distance z) {
        setX(x);
        setY(y);
        setZ(z);
    }

    /**
     * Gets norm of position expressed in meters (m), which represents the distance to
     * Earth's center of mass.
     *
     * @return position norm expressed in meters (m).
     */
    public double getNorm() {
        return Math.sqrt(mX * mX + mY * mY + mZ * mZ);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @param result instance where result will be stored.
     */
    public void getNormAsDistance(final Distance result) {
        result.setValue(getNorm());
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @return position norm.
     */
    public Distance getNormAsDistance() {
        return new Distance(getNorm(), DistanceUnit.METER);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final ECEFPosition output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFPosition input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;
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
        return Objects.hash(mX, mY, mZ);
    }

    /**
     * Checks if provided object is an ECEFPosition having exactly the same contents
     * as this instance.
     *
     * @param o Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        final ECEFPosition other = (ECEFPosition) o;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final ECEFPosition other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other instance to be compared.
     * @param threshold maximum difference allowed between position coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final ECEFPosition other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final ECEFPosition result = (ECEFPosition)super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts distance instance into meters value.
     *
     * @param distance instance to be converted.
     * @return converted value.
     */
    private double convertDistance(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }
}
