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
 * Contains GNSS measurement data of a satellite.
 */
public class GNSSMeasurement implements Serializable, Cloneable {

    /**
     * Pseudo-range measurement expressed in meters (m).
     */
    private double pseudoRange;

    /**
     * Pseudo-range rate measurement expressed in meters per second (m/s).
     */
    private double pseudoRate;

    /**
     * X coordinate of satellite ECEF position expressed in meters (m).
     */
    private double x;

    /**
     * Y coordinate of satellite ECEF position expressed in meters (m).
     */
    private double y;

    /**
     * Z coordinate of satellite ECEF position expressed in meters (m).
     */
    private double z;

    /**
     * X coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double vx;

    /**
     * Y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double vy;

    /**
     * Z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double vz;

    /**
     * Constructor.
     */
    public GNSSMeasurement() {
    }

    /**
     * Constructor.
     *
     * @param pseudoRange pseudo-range measurement expressed in meters (m).
     * @param pseudoRate  pseudo-range rate measurement expressed in meters per second (m/s).
     * @param x           x coordinate of satellite ECEF position expressed in meters (m).
     * @param y           y coordinate of satellite ECEF position expressed in meters (m).
     * @param z           z coordinate of satellite ECEF position expressed in meters (m).
     * @param vx          x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     * @param vy          y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     * @param vz          z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public GNSSMeasurement(final double pseudoRange, final double pseudoRate,
                           final double x, final double y, final double z,
                           final double vx, final double vy, final double vz) {
        setPseudoRange(pseudoRange);
        setPseudoRate(pseudoRate);
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange pseudo-range measurement.
     * @param pseudoRate  pseudo-range rate measurement.
     * @param x           x coordinate of satellite ECEF position.
     * @param y           y coordinate of satellite ECEF position.
     * @param z           z coordinate of satellite ECEF position.
     * @param vx          x coordinate of satellite ECEF velocity.
     * @param vy          y coordinate of satellite ECEF velocity.
     * @param vz          z coordinate of satellite ECEF velocity.
     */
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate,
                           final Distance x, final Distance y, final Distance z,
                           final Speed vx, final Speed vy, final Speed vz) {
        setPseudoRangeDistance(pseudoRange);
        setPseudoRateSpeed(pseudoRate);
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange pseudo-range measurement.
     * @param pseudoRate  pseudo-range rate measurement.
     * @param position    satellite ECEF position expressed in meters (m).
     * @param vx          x coordinate of satellite ECEF velocity.
     * @param vy          y coordinate of satellite ECEF velocity.
     * @param vz          z coordinate of satellite ECEF velocity.
     */
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate, final Point3D position,
                           final Speed vx, final Speed vy, final Speed vz) {
        setPseudoRangeDistance(pseudoRange);
        setPseudoRateSpeed(pseudoRate);
        setPosition(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange pseudo-range measurement expressed in meters (m).
     * @param pseudoRate  pseudo-range rate measurement expressed in meters per second (m/s).
     * @param position    satellite ECEF position.
     * @param velocity    satellite ECEF velocity.
     */
    public GNSSMeasurement(final double pseudoRange, final double pseudoRate, final ECEFPosition position,
                           final ECEFVelocity velocity) {
        setPseudoRange(pseudoRange);
        setPseudoRate(pseudoRate);
        setEcefPosition(position);
        setEcefVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange         pseudo-range measurement expressed in meters (m).
     * @param pseudoRate          pseudo-range rate measurement expressed in meters per second (m/s).
     * @param positionAndVelocity satellite ECEF position and velocity.
     */
    public GNSSMeasurement(final double pseudoRange, final double pseudoRate,
                           final ECEFPositionAndVelocity positionAndVelocity) {
        setPseudoRange(pseudoRange);
        setPseudoRate(pseudoRate);
        setPositionAndVelocity(positionAndVelocity);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange pseudo-range measurement.
     * @param pseudoRate  pseudo-range rate measurement.
     * @param position    satellite ECEF position.
     * @param velocity    satellite ECEF velocity.
     */
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate, final ECEFPosition position,
                           final ECEFVelocity velocity) {
        setPseudoRangeDistance(pseudoRange);
        setPseudoRateSpeed(pseudoRate);
        setEcefPosition(position);
        setEcefVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param pseudoRange         pseudo-range measurement.
     * @param pseudoRate          pseudo-range rate measurement.
     * @param positionAndVelocity satellite ECEF position and velocity.
     */
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate,
                           final ECEFPositionAndVelocity positionAndVelocity) {
        setPseudoRangeDistance(pseudoRange);
        setPseudoRateSpeed(pseudoRate);
        setPositionAndVelocity(positionAndVelocity);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public GNSSMeasurement(final GNSSMeasurement input) {
        copyFrom(input);
    }

    /**
     * Gets pseudo-range measurement expressed in meters (m).
     *
     * @return pseudo-range measurement expressed in meters (m).
     */
    public double getPseudoRange() {
        return pseudoRange;
    }

    /**
     * Sets pseudo-range measurement expressed in meters (m).
     *
     * @param pseudoRange pseudo-range measurement expressed in meters (m).
     */
    public void setPseudoRange(final double pseudoRange) {
        this.pseudoRange = pseudoRange;
    }

    /**
     * Gets pseudo-range measurement.
     *
     * @param result instance where pseudo-range measurement will be stored.
     */
    public void getPseudoRangeDistance(final Distance result) {
        result.setValue(pseudoRange);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement.
     *
     * @return pseudo-range measurement.
     */
    public Distance getPseudoRangeDistance() {
        return new Distance(pseudoRange, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement.
     *
     * @param pseudoRange pseudo-range measurement.
     */
    public void setPseudoRangeDistance(final Distance pseudoRange) {
        this.pseudoRange = DistanceConverter.convert(pseudoRange.getValue().doubleValue(), pseudoRange.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement expressed in meters per second (m/s).
     *
     * @return pseudo-range rate measurement expressed in meters per second (m/s).
     */
    public double getPseudoRate() {
        return pseudoRate;
    }

    /**
     * Sets pseudo-range rate measurement expressed in meters per second (m/s).
     *
     * @param pseudoRate pseudo-range rate measurement expressed in meters per second (m/s).
     */
    public void setPseudoRate(final double pseudoRate) {
        this.pseudoRate = pseudoRate;
    }

    /**
     * Gets pseudo-range rate measurement.
     *
     * @param result instance where pseudo-range rate measurement will be stored.
     */
    public void getPseudoRateSpeed(final Speed result) {
        result.setValue(pseudoRate);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement.
     *
     * @return pseudo-range rate measurement.
     */
    public Speed getPseudoRateSpeed() {
        return new Speed(pseudoRate, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets pseudo-range rate measurement.
     *
     * @param pseudoRate pseudo-range rate measurement.
     */
    public void setPseudoRateSpeed(final Speed pseudoRate) {
        this.pseudoRate = SpeedConverter.convert(pseudoRate.getValue().doubleValue(), pseudoRate.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return x coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getX() {
        return x;
    }

    /**
     * Sets x coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param x x coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets x coordinate of satellite ECEF position.
     *
     * @param result instance where x coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(x);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of satellite ECEF position.
     *
     * @return x coordinate of satellite ECEF position.
     */
    public Distance getDistanceX() {
        return new Distance(x, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of satellite ECEF position.
     *
     * @param x x coordinate of satellite ECEF position.
     */
    public void setDistanceX(final Distance x) {
        this.x = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return y coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getY() {
        return y;
    }

    /**
     * Sets y coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param y y coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets y coordinate of satellite ECEF position.
     *
     * @param result instance where y coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(y);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of satellite ECEF position.
     *
     * @return y coordinate of satellite ECEF position.
     */
    public Distance getDistanceY() {
        return new Distance(y, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of satellite ECEF position.
     *
     * @param y y coordinate of satellite ECEF position.
     */
    public void setDistanceY(final Distance y) {
        this.y = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return z coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets z coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param z z coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setZ(final double z) {
        this.z = z;
    }

    /**
     * Gets z coordinate of satellite ECEF position.
     *
     * @param result instance where z coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(z);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of satellite ECEF position.
     *
     * @return z coordinate of satellite ECEF position.
     */
    public Distance getDistanceZ() {
        return new Distance(z, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of satellite ECEF position.
     *
     * @param z z coordinate of satellite ECEF position.
     */
    public void setDistanceZ(final Distance z) {
        this.z = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets coordinates of satellite ECEF position expressed in meters (m).
     *
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Sets coordinates of satellite ECEF position.
     *
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     */
    public void setPositionCoordinates(final Distance x, final Distance y, final Distance z) {
        setDistanceX(x);
        setDistanceY(y);
        setDistanceZ(z);
    }

    /**
     * Gets satellite ECEF position expressed in meters (m).
     *
     * @param result instance where satellite ECEF position will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(x, y, z);
    }

    /**
     * Gets satellite ECEF position expressed in meters (m).
     *
     * @return ECEF position of satellite.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(x, y, z);
    }

    /**
     * Sets ECEF position of satellite expressed in meters (m).
     *
     * @param position ECEF position of satellite.
     */
    public void setPosition(final Point3D position) {
        x = position.getInhomX();
        y = position.getInhomY();
        z = position.getInhomZ();
    }

    /**
     * Gets ECEF position of satellite.
     *
     * @param result instance where ECEF position of satellite will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(x, y, z);
    }

    /**
     * Gets ECEF position of satellite.
     *
     * @return ECEF position of satellite.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(x, y, z);
    }

    /**
     * Sets ECEF position of satellite.
     *
     * @param ecefPosition ECEF position of satellite.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        x = ecefPosition.getX();
        y = ecefPosition.getY();
        z = ecefPosition.getZ();
    }

    /**
     * Gets x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVx() {
        return vx;
    }

    /**
     * Sets x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVx(final double vx) {
        this.vx = vx;
    }

    /**
     * Gets x coordinate of satellite ECEF velocity.
     *
     * @param result instance where x coordinate of satellite ECEF velocity will
     *               be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(vx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of satellite ECEF velocity.
     *
     * @return x coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedX() {
        return new Speed(vx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of satellite ECEF velocity.
     *
     * @param speedX x coordinate of satellite ECEF velocity.
     */
    public void setSpeedX(final Speed speedX) {
        vx = SpeedConverter.convert(speedX.getValue().doubleValue(), speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVy() {
        return vy;
    }

    /**
     * Sets y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vy y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVy(final double vy) {
        this.vy = vy;
    }

    /**
     * Gets y coordinate of satellite ECEF velocity.
     *
     * @param result instance where y coordinate of satellite ECEF velocity will
     *               be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(vy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite ECEF velocity.
     *
     * @return y coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedY() {
        return new Speed(vy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of satellite ECEF velocity.
     *
     * @param speedY y coordinate of satellite ECEF velocity.
     */
    public void setSpeedY(final Speed speedY) {
        vy = SpeedConverter.convert(speedY.getValue().doubleValue(), speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVz() {
        return vz;
    }

    /**
     * Sets z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vz z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVz(final double vz) {
        this.vz = vz;
    }

    /**
     * Gets z coordinate of satellite ECEF velocity.
     *
     * @param result z coordinate of satellite ECEF velocity.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(vz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite ECEF velocity.
     *
     * @return z coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(vz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of satellite ECEF velocity.
     *
     * @param speedZ z coordinate of satellite ECEF velocity.
     */
    public void setSpeedZ(final Speed speedZ) {
        vz = SpeedConverter.convert(speedZ.getValue().doubleValue(), speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets ECEF coordinates of satellite velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of satellite ECEF velocity.
     * @param vy y coordinate of satellite ECEF velocity.
     * @param vz z coordinate of satellite ECEF velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    /**
     * Sets ECEF coordinates of satellite velocity.
     *
     * @param speedX x coordinate of satellite ECEF velocity.
     * @param speedY y coordinate of satellite ECEF velocity.
     * @param speedZ z coordinate of satellite ECEF velocity.
     */
    public void setVelocityCoordinates(final Speed speedX, final Speed speedY, final Speed speedZ) {
        setSpeedX(speedX);
        setSpeedY(speedY);
        setSpeedZ(speedZ);
    }

    /**
     * Gets ECEF velocity of satellite.
     *
     * @param result instance where ECEF velocity of satellite will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(vx, vy, vz);
    }

    /**
     * Gets ECEF velocity of satellite.
     *
     * @return ECEF velocity of satellite.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(vx, vy, vz);
    }

    /**
     * Sets ECEF velocity of satellite.
     *
     * @param ecefVelocity ECEF velocity of satellite.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        vx = ecefVelocity.getVx();
        vy = ecefVelocity.getVy();
        vz = ecefVelocity.getVz();
    }

    /**
     * Gets ECEF position and velocity of satellite.
     *
     * @param result instance where position and velocity will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(x, y, z);
        result.setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Gets ECEF position and velocity of satellite.
     *
     * @return ECEF position and velocity of satellite.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
    }

    /**
     * Sets ECEF position and velocity of satellite.
     *
     * @param positionAndVelocity ECEF position and velocity of satellite.
     */
    public void setPositionAndVelocity(final ECEFPositionAndVelocity positionAndVelocity) {
        x = positionAndVelocity.getX();
        y = positionAndVelocity.getY();
        z = positionAndVelocity.getZ();

        vx = positionAndVelocity.getVx();
        vy = positionAndVelocity.getVy();
        vz = positionAndVelocity.getVz();
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSMeasurement output) {
        output.pseudoRange = pseudoRange;
        output.pseudoRate = pseudoRate;

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
    public void copyFrom(final GNSSMeasurement input) {
        pseudoRange = input.pseudoRange;
        pseudoRate = input.pseudoRate;

        x = input.x;
        y = input.y;
        z = input.z;

        vx = input.vx;
        vy = input.vy;
        vz = input.vz;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
    }

    /**
     * Checks if provided object is a GNSSMeasurement having exactly the same contents
     * as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == this) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        final var other = (GNSSMeasurement) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final GNSSMeasurement other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final GNSSMeasurement other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(pseudoRange - other.pseudoRange) <= threshold
                && Math.abs(pseudoRate - other.pseudoRate) <= threshold
                && Math.abs(x - other.x) <= threshold
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
        final var result = (GNSSMeasurement) super.clone();
        copyTo(result);
        return result;
    }
}
