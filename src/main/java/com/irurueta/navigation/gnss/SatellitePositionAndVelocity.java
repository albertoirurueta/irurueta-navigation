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
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains satellite position and velocity.
 */
public class SatellitePositionAndVelocity implements Serializable, Cloneable {

    /**
     * Cartesian x coordinate of satellite position resolved in ECEF axes and expressed
     * in meters (m).
     */
    private double mX;

    /**
     * Cartesian y coordinate of satellite position resolved in ECEF axes and expressed
     * in meters (m).
     */
    private double mY;

    /**
     * Cartesian z coordinate of satellite position resolved in ECEF axes and expressed
     * in meters(m).
     */
    private double mZ;

    /**
     * X coordinate of satellite velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    private double mVx;

    /**
     * Y coordinate of satellite velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    private double mVy;

    /**
     * Z coordinate of satellite velocity resolved in ECEF axes and expressed in meters
     * per second (m/s).
     */
    private double mVz;

    /**
     * Constructor.
     */
    public SatellitePositionAndVelocity() {
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of satellite ECEF position expressed in
     *          meters (m).
     * @param y cartesian y coordinate of satellite ECEF position expressed in
     *          meters (m).
     * @param z cartesian z coordinate of satellite ECEF position expressed in
     *          meters (m).
     */
    public SatellitePositionAndVelocity(final double x, final double y, final double z) {
        setPositionCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of satellite ECEF position.
     * @param y cartesian y coordinate of satellite ECEF position.
     * @param z cartesian z coordinate of satellite ECEF position.
     */
    public SatellitePositionAndVelocity(final Distance x, final Distance y,
                                        final Distance z) {
        setPositionDistanceCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition satellite ECEF position.
     */
    public SatellitePositionAndVelocity(final ECEFPosition ecefPosition) {
        setEcefPosition(ecefPosition);
    }

    /**
     * Constructor.
     *
     * @param position satellite position.
     */
    public SatellitePositionAndVelocity(final Point3D position) {
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param vx x coordinate of satellite velocity resolved in ECEF axes.
     * @param vy y coordinate of satellite velocity resolved in ECEF axes.
     * @param vz z coordinate of satellite velocity resolved in ECEF axes.
     */
    public SatellitePositionAndVelocity(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefVelocity satellite velocity.
     */
    public SatellitePositionAndVelocity(final ECEFVelocity ecefVelocity) {
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param y  cartesian y coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param z  cartesian z coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param vx x coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vy y coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vz z coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     */
    public SatellitePositionAndVelocity(final double x, final double y, final double z,
                                        final double vx, final double vy, final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param y  cartesian y coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param z  cartesian z coordinate of satellite ECEF position expressed in
     *           meters (m).
     * @param vx x coordinate of satellite velocity resolved in ECEF axes.
     * @param vy y coordinate of satellite velocity resolved in ECEF axes.
     * @param vz z coordinate of satellite velocity resolved in ECEF axes.
     */
    public SatellitePositionAndVelocity(final double x, final double y, final double z,
                                        final Speed vx, final Speed vy, final Speed vz) {
        this(x, y, z);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x            cartesian x coordinate of satellite ECEF position expressed in
     *                     meters (m).
     * @param y            cartesian y coordinate of satellite ECEF position expressed in
     *                     meters (m).
     * @param z            cartesian z coordinate of satellite ECEF position expressed in
     *                     meters (m).
     * @param ecefVelocity satellite velocity.
     */
    public SatellitePositionAndVelocity(final double x, final double y, final double z,
                                        final ECEFVelocity ecefVelocity) {
        this(x, y, z);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of satellite ECEF position.
     * @param y  cartesian y coordinate of satellite ECEF position.
     * @param z  cartesian z coordinate of satellite ECEF position.
     * @param vx x coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vy y coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     * @param vz z coordinate of satellite velocity resolved in ECEF axes and expressed
     *           in meters per second (m/s).
     */
    public SatellitePositionAndVelocity(final Distance x, final Distance y,
                                        final Distance z, final double vx, final double vy,
                                        final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of satellite ECEF position.
     * @param y  cartesian y coordinate of satellite ECEF position.
     * @param z  cartesian z coordinate of satellite ECEF position.
     * @param vx x coordinate of satellite velocity resolved in ECEF axes.
     * @param vy y coordinate of satellite velocity resolved in ECEF axes.
     * @param vz z coordinate of satellite velocity resolved in ECEF axes.
     */
    public SatellitePositionAndVelocity(final Distance x, final Distance y,
                                        final Distance z, final Speed vx, final Speed vy,
                                        final Speed vz) {
        this(x, y, z);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param x            cartesian x coordinate of satellite ECEF position.
     * @param y            cartesian y coordinate of satellite ECEF position.
     * @param z            cartesian z coordinate of satellite ECEF position.
     * @param ecefVelocity satellite velocity.
     */
    public SatellitePositionAndVelocity(final Distance x, final Distance y,
                                        final Distance z,
                                        final ECEFVelocity ecefVelocity) {
        this(x, y, z);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition satellite ECEF position.
     * @param vx           x coordinate of satellite velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     * @param vy           y coordinate of satellite velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     * @param vz           z coordinate of satellite velocity resolved in ECEF axes and expressed
     *                     in meters per second (m/s).
     */
    public SatellitePositionAndVelocity(final ECEFPosition ecefPosition,
                                        final double vx, final double vy, final double vz) {
        this(ecefPosition);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition satellite ECEF position.
     * @param vx           x coordinate of satellite velocity resolved in ECEF axes.
     * @param vy           y coordinate of satellite velocity resolved in ECEF axes.
     * @param vz           z coordinate of satellite velocity resolved in ECEF axes.
     */
    public SatellitePositionAndVelocity(final ECEFPosition ecefPosition,
                                        final Speed vx, final Speed vy, final Speed vz) {
        this(ecefPosition);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param ecefPosition satellite ECEF position.
     * @param ecefVelocity satellite velocity.
     */
    public SatellitePositionAndVelocity(final ECEFPosition ecefPosition,
                                        final ECEFVelocity ecefVelocity) {
        this(ecefPosition);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Constructor.
     *
     * @param position satellite position.
     * @param vx       x coordinate of satellite velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     * @param vy       y coordinate of satellite velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     * @param vz       z coordinate of satellite velocity resolved in ECEF axes and expressed
     *                 in meters per second (m/s).
     */
    public SatellitePositionAndVelocity(final Point3D position,
                                        final double vx, final double vy, final double vz) {
        this(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position satellite position.
     * @param vx       x coordinate of satellite velocity resolved in ECEF axes.
     * @param vy       y coordinate of satellite velocity resolved in ECEF axes.
     * @param vz       z coordinate of satellite velocity resolved in ECEF axes.
     */
    public SatellitePositionAndVelocity(final Point3D position,
                                        final Speed vx, final Speed vy, final Speed vz) {
        this(position);
        setSpeedCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position     satellite position.
     * @param ecefVelocity satellite velocity.
     */
    public SatellitePositionAndVelocity(final Point3D position,
                                        final ECEFVelocity ecefVelocity) {
        this(position);
        setEcefVelocity(ecefVelocity);
    }

    /**
     * Gets cartesian x coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian x coordinate of satellite position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets cartesian x coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param x cartesian x coordinate of satellite position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets cartesian y coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian y coordinate of satellite position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets cartesian y coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param y cartesian y coordinate of satellite position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets cartesian z coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @return cartesian z coordinate of satellite position resolved in ECEF axes
     * and expressed in meters (m).
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets cartesian z coordinate of satellite position resolved in ECEF axes and
     * expressed in meters (m).
     *
     * @param z cartesian z coordinate of satellite position resolved in ECEF axes
     *          and expressed in meters (m).
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets satellite ECEF position expressed in meters (m).
     *
     * @param x cartesian x coordinate of satellite position.
     * @param y cartesian y coordinate of satellite position.
     * @param z cartesian z coordinate of satellite position.
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Gets cartesian x coordinate of satellite position resolved in ECEF axes.
     *
     * @param result instance where cartesian x coordinate of satellite position will
     *               be stored.
     */
    public void getXDistance(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of satellite position resolved in ECEF axes.
     *
     * @return cartesian x coordinate of satellite position.
     */
    public Distance getXDistance() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of satellite position resolved in ECEF axes.
     *
     * @param x cartesian x coordinate of satellite position.
     */
    public void setXDistance(final Distance x) {
        mX = DistanceConverter.convert(x.getValue().doubleValue(),
                x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of satellite position resolved in ECEF axes.
     *
     * @param result instance where cartesian y coordinate of satellite position will
     *               be stored.
     */
    public void getYDistance(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of satellite position resolved in ECEF axes.
     *
     * @return cartesian y coordinate of satellite position.
     */
    public Distance getYDistance() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of satellite position resolved in ECEF axes.
     *
     * @param y cartesian y coordinate of satellite position.
     */
    public void setYDistance(final Distance y) {
        mY = DistanceConverter.convert(y.getValue().doubleValue(),
                y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of satellite position resolved in ECEF axes.
     *
     * @param result instance where cartesian z coordinate of satellite position will
     *               be stored.
     */
    public void getZDistance(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of satellite position resolved in ECEF axes.
     *
     * @return cartesian z coordinate of satellite position.
     */
    public Distance getZDistance() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of satellite position resolved in ECEF axes.
     *
     * @param z cartesian z coordinate of satellite position.
     */
    public void setZDistance(final Distance z) {
        mZ = DistanceConverter.convert(z.getValue().doubleValue(),
                z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets satellite position resolved in ECEF axes.
     *
     * @param x cartesian x coordinate of satellite position.
     * @param y cartesian y coordinate of satellite position.
     * @param z cartesian z coordinate of satellite position.
     */
    public void setPositionDistanceCoordinates(final Distance x, final Distance y,
                                               final Distance z) {
        setXDistance(x);
        setYDistance(y);
        setZDistance(z);
    }

    /**
     * Gets ECEF satellite position.
     *
     * @param result instance where satellite ECEF position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(mX, mY, mZ);
    }

    /**
     * Gets ECEF satellite position.
     *
     * @return satellite ECEF position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(mX, mY, mZ);
    }

    /**
     * Sets ECEF satellite position.
     *
     * @param ecefPosition satellite ECEF position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        mX = ecefPosition.getX();
        mY = ecefPosition.getY();
        mZ = ecefPosition.getZ();
    }

    /**
     * Gets satellite position resolved in ECEF axes.
     *
     * @param result instance where satellite position will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Gets satellite position resolved in ECEF axes.
     *
     * @return satellite position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Sets satellite position resolved in ECEF axes.
     *
     * @param position satellite position.
     */
    public void setPosition(final Point3D position) {
        mX = position.getInhomX();
        mY = position.getInhomY();
        mZ = position.getInhomZ();
    }

    /**
     * Gets x coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return x coordinate of satellite velocity resolved in ECEF axes.
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vx x coordinate of satellite velocity resolved in ECEF axes.
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return y coordinate of satellite velocity resolved in ECEF axes.
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vy y coordinate of satellite velocity resolved in ECEf axes.
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @return z coordinate of satellite velocity resolved in ECEF axes.
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of satellite velocity resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vz z coordinate of satellite velocity resolved in ECEF axes.
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets satellite velocity coordinates resolved in ECEF axes and expressed in
     * meters per second (m/s).
     *
     * @param vx x coordinate of satellite velocity.
     * @param vy y coordinate of satellite velocity.
     * @param vz z coordinate of satellite velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets x coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param result instance where x coordinate of satellite velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of satellite velocity resolved in ECEF axes.
     *
     * @return x coordinate of satellite velocity.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param vx x coordinate of satellite velocity.
     */
    public void setSpeedX(final Speed vx) {
        mVx = SpeedConverter.convert(vx.getValue().doubleValue(),
                vx.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param result instance where y coordinate of satellite velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite velocity resolved in ECEF axes.
     *
     * @return y coordinate of satellite velocity.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param vy y coordinate of satellite velocity.
     */
    public void setSpeedY(final Speed vy) {
        mVy = SpeedConverter.convert(vy.getValue().doubleValue(),
                vy.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param result instance where z coordinate of satellite velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite velocity resolved in ECEF axes.
     *
     * @return z coordinate of satellite velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of satellite velocity resolved in ECEF axes.
     *
     * @param vz z coordinate of satellite velocity.
     */
    public void setSpeedZ(final Speed vz) {
        mVz = SpeedConverter.convert(vz.getValue().doubleValue(),
                vz.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinates of satellite velocity resolved in ECEF axes.
     *
     * @param vx x coordinate of satellite velocity.
     * @param vy y coordinate of satellite velocity.
     * @param vz z coordinate of satellite velocity.
     */
    public void setSpeedCoordinates(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedX(vx);
        setSpeedY(vy);
        setSpeedZ(vz);
    }

    /**
     * Gets satellite velocity resolved in ECEF axes.
     *
     * @param result instance where satellite velocity will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets satellite velocity resolved in ECEF axes.
     *
     * @return satellite velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(mVx, mVy, mVz);
    }

    /**
     * Sets satellite velocity resolved in ECEF axes.
     *
     * @param ecefVelocity satellite velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        mVx = ecefVelocity.getVx();
        mVy = ecefVelocity.getVy();
        mVz = ecefVelocity.getVz();
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final SatellitePositionAndVelocity output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;

        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final SatellitePositionAndVelocity input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;
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
        return Objects.hash(mX, mY, mZ, mVx, mVy, mVz);
    }

    /**
     * Checks if provided object is a SatellitePositionAndVelocity having exactly the
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
        SatellitePositionAndVelocity other = (SatellitePositionAndVelocity) o;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final SatellitePositionAndVelocity other) {
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
    public boolean equals(final SatellitePositionAndVelocity other,
                          final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mVx - other.mVx) <= threshold
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
        final SatellitePositionAndVelocity result = (SatellitePositionAndVelocity) super.clone();
        copyTo(result);
        return result;
    }
}
