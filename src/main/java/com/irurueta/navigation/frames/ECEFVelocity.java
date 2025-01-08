package com.irurueta.navigation.frames;

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
public class ECEFVelocity implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * X coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double vx;

    /**
     * Y coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double vy;

    /**
     * Z coordinate of velocity of body frame expressed in meters per second (m/s) with
     * respect ECEF frame, resolved along the corresponding frame axes.
     */
    private double vz;

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
        return vx;
    }

    /**
     * Sets x coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     */
    public void setVx(final double vx) {
        this.vx = vx;
    }

    /**
     * Gets y coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity.
     */
    public double getVy() {
        return vy;
    }

    /**
     * Sets y coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vy y coordinate of velocity.
     */
    public void setVy(final double vy) {
        this.vy = vy;
    }

    /**
     * Gets z coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity.
     */
    public double getVz() {
        return vz;
    }

    /**
     * Sets z coordinate of velocity of body frame expressed in meters per second (m/s)
     * resolved along ECEF-frame axes.
     *
     * @param vz z coordinate of velocity.
     */
    public void setVz(final double vz) {
        this.vz = vz;
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
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    /**
     * Gets cartesian x coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian x coordinate of velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(vx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return x coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedX() {
        return new Speed(vx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of body velocity to be set.
     */
    public void setVx(final Speed vx) {
        this.vx = convertSpeed(vx);
    }

    /**
     * Gets cartesian y coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian y coordinate of velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(vy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedY() {
        return new Speed(vy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vy y coordinate of body velocity to be set.
     */
    public void setVy(final Speed vy) {
        this.vy = convertSpeed(vy);
    }

    /**
     * Gets cartesian z coordinate of velocity of body frame resolved along ECEF-frame
     * axes.
     *
     * @param result instance where cartesian z coordinate of velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(vz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedZ() {
        return new Speed(vz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param vz z coordinate of body velocity to be set.
     */
    public void setVz(final Speed vz) {
        this.vz = convertSpeed(vz);
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
        return Math.sqrt(vx * vx + vy * vy + vz * vz);
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
        output.vx = vx;
        output.vy = vy;
        output.vz = vz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFVelocity input) {
        vx = input.vx;
        vy = input.vy;
        vz = input.vz;
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

        result[0] = vx;
        result[1] = vy;
        result[2] = vz;
    }

    /**
     * Gets velocity coordinates expressed in meters per second (m/s) as an array.
     *
     * @return array containing velocity coordinates in x,y,z order.
     */
    public double[] asArray() {
        final var result = new double[COMPONENTS];
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
    @SuppressWarnings("DuplicatedCode")
    public void asMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, vx);
        result.setElementAtIndex(1, vy);
        result.setElementAtIndex(2, vz);
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
        return Objects.hash(vx, vy, vz);
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
        final var other = (ECEFVelocity) o;
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

        return Math.abs(vx - other.vx) <= threshold
                && Math.abs(vy - other.vy) <= threshold
                && Math.abs(vz - other.vz) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (ECEFVelocity) super.clone();
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
