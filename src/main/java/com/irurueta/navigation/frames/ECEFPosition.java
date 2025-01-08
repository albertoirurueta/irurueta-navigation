/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
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
public class ECEFPosition implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * Cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double x;

    /**
     * Cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double y;

    /**
     * Cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     * along ECEF axes.
     */
    private double z;

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
        return x;
    }

    /**
     * Sets cartesian x coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param x cartesian x coordinate of body position.
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets cartesian y coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @return cartesian y coordinate of body position.
     */
    public double getY() {
        return y;
    }

    /**
     * Sets cartesian y coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param y cartesian y coordinate of body position.
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets cartesian z coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @return z coordinate of body position.
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets cartesian z coordinate of body position expressed in meters (m) with respect
     * ECEF frame, resolved along ECEF axes.
     *
     * @param z cartesian z coordinate of body position.
     */
    public void setZ(final double z) {
        this.z = z;
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
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return body position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(x, y, z);
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param result instance where position data is copied to.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(x, y, z);
    }

    /**
     * Sets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param point body position to be set.
     */
    public void setPosition(final Point3D point) {
        x = point.getInhomX();
        y = point.getInhomY();
        z = point.getInhomZ();
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian x coordinate of body position will be
     *               stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(x);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @return x coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceX() {
        return new Distance(x, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position to be set.
     */
    public void setX(final Distance x) {
        this.x = convertDistance(x);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian y coordinate of body position will be
     *               stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(y);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @return y coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceY() {
        return new Distance(y, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param y cartesian y coordinate of body position to be set.
     */
    public void setY(final Distance y) {
        this.y = convertDistance(y);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian z coordinate of body position will be
     *               stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(z);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @return z coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getDistanceZ() {
        return new Distance(z, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param z cartesian z coordinate of body position to be set.
     */
    public void setZ(final Distance z) {
        this.z = convertDistance(z);
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
        return Math.sqrt(x * x + y * y + z * z);
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
        output.x = x;
        output.y = y;
        output.z = z;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFPosition input) {
        x = input.x;
        y = input.y;
        z = input.z;
    }

    /**
     * Gets position coordinates expressed in meters (m) as an array.
     *
     * @param result array instance where position coordinates will
     *               be stored in x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = x;
        result[1] = y;
        result[2] = z;
    }

    /**
     * Gets position coordinates expressed in meters (m) as an array.
     *
     * @return array containing position coordinates in x,y,z order.
     */
    public double[] asArray() {
        final var result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets position coordinates expressed in meters (m) as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where gravity coordinates will be stored in
     *               x,y,z order.
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

        result.setElementAtIndex(0, x);
        result.setElementAtIndex(1, y);
        result.setElementAtIndex(2, z);
    }

    /**
     * Gets position coordinates expressed in meters (m) as a column matrix.
     *
     * @return position coordinates stored in x,y,z order.
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
        return Objects.hash(x, y, z);
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
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final ECEFPosition other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(x - other.x) <= threshold
                && Math.abs(y - other.y) <= threshold
                && Math.abs(z - other.z) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (ECEFPosition) super.clone();
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
