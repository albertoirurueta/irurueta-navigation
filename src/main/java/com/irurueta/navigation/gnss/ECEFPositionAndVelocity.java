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
package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains position and velocity resolved in ECEF axes.
 */
public class ECEFPositionAndVelocity implements Serializable, Cloneable {

    /**
     * Cartesian x coordinate of position resolved in ECEF axes and expressed
     * in meters (m).
     */
    double x;

    /**
     * Cartesian y coordinate of position resolved in ECEF axes and expressed
     * in meters (m).
     */
    double y;

    /**
     * Cartesian z coordinate of position resolved in ECEF axes and expressed
     * in meters(m).
     */
    double z;

    /**
     * X coordinate of velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    double vx;

    /**
     * Y coordinate of velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    double vy;

    /**
     * Z coordinate of velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    double vz;

    /**
     * Constructor.
     */
    public ECEFPositionAndVelocity() {
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of ECEF position expressed in
     *          meters (m).
     * @param y cartesian y coordinate of ECEF position expressed in
     *          meters (m).
     * @param z cartesian z coordinate of ECEF position expressed in
     *          meters (m).
     */
    public ECEFPositionAndVelocity(final double x, final double y, final double z) {
        setPositionCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of ECEF position.
     * @param y cartesian y coordinate of ECEF position.
     * @param z cartesian z coordinate of ECEF position.
     */
    public ECEFPositionAndVelocity(final Distance x, final Distance y, final Distance z) {
        setPositionDistanceCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition ECEF position.
     */
    public ECEFPositionAndVelocity(final ECEFPosition ecefPosition) {
        setEcefPosition(ecefPosition);
    }

    /**
     * Constructor.
     *
     * @param position position to be set.
     */
    public ECEFPositionAndVelocity(final Point3D position) {
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param vx x coordinate of velocity resolved in ECEF axes.
     * @param vy y coordinate of velocity resolved in ECEF axes.
     * @param vz z coordinate of velocity resolved in ECEF axes.
     */
    public ECEFPositionAndVelocity(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefVelocity velocity to be set.
     */
    public ECEFPositionAndVelocity(final ECEFVelocity ecefVelocity) {
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of ECEF position expressed in
     *           meters (m).
     * @param y  cartesian y coordinate of ECEF position expressed in
     *           meters (m).
     * @param z  cartesian z coordinate of ECEF position expressed in
     *           meters (m).
     * @param vx x coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vy y coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vz z coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     */
    public ECEFPositionAndVelocity(final double x, final double y, final double z,
                                   final double vx, final double vy, final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of ECEF position expressed in
     *           meters (m).
     * @param y  cartesian y coordinate of ECEF position expressed in
     *           meters (m).
     * @param z  cartesian z coordinate of ECEF position expressed in
     *           meters (m).
     * @param vx x coordinate of velocity resolved in ECEF axes.
     * @param vy y coordinate of velocity resolved in ECEF axes.
     * @param vz z coordinate of velocity resolved in ECEF axes.
     */
    public ECEFPositionAndVelocity(final double x, final double y, final double z,
                                   final Speed vx, final Speed vy, final Speed vz) {
        this(x, y, z);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x            cartesian x coordinate of ECEF position expressed in
     *                     meters (m).
     * @param y            cartesian y coordinate of ECEF position expressed in
     *                     meters (m).
     * @param z            cartesian z coordinate of ECEF position expressed in
     *                     meters (m).
     * @param ecefVelocity satellite velocity.
     */
    public ECEFPositionAndVelocity(final double x, final double y, final double z, final ECEFVelocity ecefVelocity) {
        this(x, y, z);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of ECEF position.
     * @param y  cartesian y coordinate of ECEF position.
     * @param z  cartesian z coordinate of ECEF position.
     * @param vx x coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vy y coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vz z coordinate of velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     */
    public ECEFPositionAndVelocity(final Distance x, final Distance y, final Distance z,
                                   final double vx, final double vy, final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of ECEF position.
     * @param y  cartesian y coordinate of ECEF position.
     * @param z  cartesian z coordinate of ECEF position.
     * @param vx x coordinate of velocity resolved in ECEF axes.
     * @param vy y coordinate of velocity resolved in ECEF axes.
     * @param vz z coordinate of velocity resolved in ECEF axes.
     */
    public ECEFPositionAndVelocity(final Distance x, final Distance y, final Distance z,
                                   final Speed vx, final Speed vy, final Speed vz) {
        this(x, y, z);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x            cartesian x coordinate of ECEF position.
     * @param y            cartesian y coordinate of ECEF position.
     * @param z            cartesian z coordinate of ECEF position.
     * @param ecefVelocity satellite velocity.
     */
    public ECEFPositionAndVelocity(final Distance x, final Distance y, final Distance z,
                                   final ECEFVelocity ecefVelocity) {
        this(x, y, z);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition ECEF position to be set.
     * @param vx           x coordinate of velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     * @param vy           y coordinate of velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     * @param vz           z coordinate of velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     */
    public ECEFPositionAndVelocity(final ECEFPosition ecefPosition,
                                   final double vx, final double vy, final double vz) {
        this(ecefPosition);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition ECEF position to be set.
     * @param vx           x coordinate of velocity resolved in ECEF axes.
     * @param vy           y coordinate of velocity resolved in ECEF axes.
     * @param vz           z coordinate of velocity resolved in ECEF axes.
     */
    public ECEFPositionAndVelocity(final ECEFPosition ecefPosition,
                                   final Speed vx, final Speed vy, final Speed vz) {
        this(ecefPosition);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition ECEF position to be set.
     * @param ecefVelocity velocity to be set.
     */
    public ECEFPositionAndVelocity(final ECEFPosition ecefPosition, final ECEFVelocity ecefVelocity) {
        this(ecefPosition);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param position position to be set.
     * @param vx       x coordinate of velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     * @param vy       y coordinate of velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     * @param vz       z coordinate of velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     */
    public ECEFPositionAndVelocity(final Point3D position, final double vx, final double vy, final double vz) {
        this(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position position to be set.
     * @param vx       x coordinate of velocity resolved in ECEF axes.
     * @param vy       y coordinate of velocity resolved in ECEF axes.
     * @param vz       z coordinate of velocity resolved in ECEF axes.
     */
    public ECEFPositionAndVelocity(final Point3D position, final Speed vx, final Speed vy, final Speed vz) {
        this(position);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position     position to be set.
     * @param ecefVelocity velocity to be set.
     */
    public ECEFPositionAndVelocity(final Point3D position, final ECEFVelocity ecefVelocity) {
        this(position);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public ECEFPositionAndVelocity(final ECEFPositionAndVelocity input) {
        copyFrom(input);
    }

    /**
     * Gets cartesian x coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian x coordinate of position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getX() {
        return x;
    }

    /**
     * Sets cartesian x coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param x cartesian x coordinate of position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets cartesian y coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian y coordinate of position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getY() {
        return y;
    }

    /**
     * Sets cartesian y coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param y cartesian y coordinate of position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets cartesian z coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian z coordinate of position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets cartesian z coordinate of position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param z cartesian z coordinate of position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setZ(final double z) {
        this.z = z;
    }

    /**
     * Sets ECEF position expressed in meters (m).
     *
     * @param x cartesian x coordinate of position.
     * @param y cartesian y coordinate of position.
     * @param z cartesian z coordinate of position.
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Gets cartesian x coordinate of position resolved in ECEF axes.
     *
     * @param result instance where cartesian x coordinate of position will
     *               be stored.
     */
    public void getXDistance(final Distance result) {
        result.setValue(x);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of position resolved in ECEF axes.
     *
     * @return cartesian x coordinate of position.
     */
    public Distance getXDistance() {
        return new Distance(x, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of position resolved in ECEF axes.
     *
     * @param x cartesian x coordinate of position.
     */
    public void setXDistance(final Distance x) {
        this.x = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of position resolved in ECEF axes.
     *
     * @param result instance where cartesian y coordinate of position will
     *               be stored.
     */
    public void getYDistance(final Distance result) {
        result.setValue(y);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of position resolved in ECEF axes.
     *
     * @return cartesian y coordinate of position.
     */
    public Distance getYDistance() {
        return new Distance(y, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of position resolved in ECEF axes.
     *
     * @param y cartesian y coordinate of position.
     */
    public void setYDistance(final Distance y) {
        this.y = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of position resolved in ECEF axes.
     *
     * @param result instance where cartesian z coordinate of position will
     *               be stored.
     */
    public void getZDistance(final Distance result) {
        result.setValue(z);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of position resolved in ECEF axes.
     *
     * @return cartesian z coordinate of position.
     */
    public Distance getZDistance() {
        return new Distance(z, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of position resolved in ECEF axes.
     *
     * @param z cartesian z coordinate of position.
     */
    public void setZDistance(final Distance z) {
        this.z = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets position resolved in ECEF axes.
     *
     * @param x cartesian x coordinate of position.
     * @param y cartesian y coordinate of position.
     * @param z cartesian z coordinate of position.
     */
    public void setPositionDistanceCoordinates(final Distance x, final Distance y, final Distance z) {
        setXDistance(x);
        setYDistance(y);
        setZDistance(z);
    }

    /**
     * Gets ECEF position.
     *
     * @param result instance where ECEF position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(x, y, z);
    }

    /**
     * Gets ECEF position.
     *
     * @return ECEF position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(x, y, z);
    }

    /**
     * Sets ECEF position.
     *
     * @param ecefPosition ECEF position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        x = ecefPosition.getX();
        y = ecefPosition.getY();
        z = ecefPosition.getZ();
    }

    /**
     * Gets position resolved in ECEF axes.
     *
     * @param result instance where position will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(x, y, z);
    }

    /**
     * Gets position resolved in ECEF axes.
     *
     * @return position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(x, y, z);
    }

    /**
     * Sets position resolved in ECEF axes.
     *
     * @param position position.
     */
    public void setPosition(final Point3D position) {
        x = position.getInhomX();
        y = position.getInhomY();
        z = position.getInhomZ();
    }

    /**
     * Gets x coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return x coordinate of velocity resolved in ECEF axes.
     */
    public double getVx() {
        return vx;
    }

    /**
     * Sets x coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vx x coordinate of velocity resolved in ECEF axes.
     */
    public void setVx(final double vx) {
        this.vx = vx;
    }

    /**
     * Gets y coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return y coordinate of velocity resolved in ECEF axes.
     */
    public double getVy() {
        return vy;
    }

    /**
     * Sets y coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vy y coordinate of velocity resolved in ECEf axes.
     */
    public void setVy(final double vy) {
        this.vy = vy;
    }

    /**
     * Gets z coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return z coordinate of velocity resolved in ECEF axes.
     */
    public double getVz() {
        return vz;
    }

    /**
     * Sets z coordinate of velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vz z coordinate of velocity resolved in ECEF axes.
     */
    public void setVz(final double vz) {
        this.vz = vz;
    }

    /**
     * Sets velocity coordinates resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vx x coordinate of velocity.
     * @param vy y coordinate of velocity.
     * @param vz z coordinate of velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    /**
     * Gets x coordinate of velocity resolved in ECEF axes.
     *
     * @param result instance where x coordinate of velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(vx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity resolved in ECEF axes.
     *
     * @return x coordinate of velocity.
     */
    public Speed getSpeedX() {
        return new Speed(vx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of velocity resolved in ECEF axes.
     *
     * @param vx x coordinate of velocity.
     */
    public void setSpeedX(final Speed vx) {
        this.vx = SpeedConverter.convert(vx.getValue().doubleValue(), vx.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity resolved in ECEF axes.
     *
     * @param result instance where y coordinate of velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(vy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity resolved in ECEF axes.
     *
     * @return y coordinate of velocity.
     */
    public Speed getSpeedY() {
        return new Speed(vy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of velocity resolved in ECEF axes.
     *
     * @param vy y coordinate of velocity.
     */
    public void setSpeedY(final Speed vy) {
        this.vy = SpeedConverter.convert(vy.getValue().doubleValue(), vy.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity resolved in ECEF axes.
     *
     * @param result instance where z coordinate of velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(vz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity resolved in ECEF axes.
     *
     * @return z coordinate of velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(vz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of velocity resolved in ECEF axes.
     *
     * @param vz z coordinate of velocity.
     */
    public void setSpeedZ(final Speed vz) {
        this.vz = SpeedConverter.convert(vz.getValue().doubleValue(), vz.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinates of velocity resolved in ECEF axes.
     *
     * @param vx x coordinate of velocity.
     * @param vy y coordinate of velocity.
     * @param vz z coordinate of velocity.
     */
    public void setSpeedCoordinates(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedX(vx);
        setSpeedY(vy);
        setSpeedZ(vz);
    }

    /**
     * Gets velocity resolved in ECEF axes.
     *
     * @param result instance where velocity will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(vx, vy, vz);
    }

    /**
     * Gets velocity resolved in ECEF axes.
     *
     * @return velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(vx, vy, vz);
    }

    /**
     * Sets velocity resolved in ECEF axes.
     *
     * @param ecefVelocity velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        vx = ecefVelocity.getVx();
        vy = ecefVelocity.getVy();
        vz = ecefVelocity.getVz();
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final ECEFPositionAndVelocity output) {
        output.x = x;
        output.y = y;
        output.z = z;

        output.vx = vx;
        output.vy = vy;
        output.vz = vz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFPositionAndVelocity input) {
        x = input.x;
        y = input.y;
        z = input.z;

        vx = input.vx;
        vy = input.vy;
        vz = input.vz;
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
        return Objects.hash(x, y, z, vx, vy, vz);
    }

    /**
     * Checks if provided object is a ECEFPositionAndVelocity having exactly the
     * same contents as this instance.
     *
     * @param o object to be compared.
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
        final var other = (ECEFPositionAndVelocity) o;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final ECEFPositionAndVelocity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position and velocity
     *                  coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final ECEFPositionAndVelocity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(x - other.x) <= threshold
                && Math.abs(y - other.y) <= threshold
                && Math.abs(z - other.z) <= threshold
                && Math.abs(vx - other.vx) <= threshold
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
        final var result = (ECEFPositionAndVelocity) super.clone();
        copyTo(result);
        return result;
    }
}
