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
 * Contains GNSS measurement data of a satellite.
 */
@SuppressWarnings("WeakerAccess")
public class GNSSMeasurement implements Serializable, Cloneable {

    /**
     * Pseudo-range measurement expressed in meters (m).
     */
    private double mPseudoRange;

    /**
     * Pseudo-range rate measurement expressed in meters per second (m/s).
     */
    private double mPseudoRate;

    /**
     * X coordinate of satellite ECEF position expressed in meters (m).
     */
    private double mX;

    /**
     * Y coordinate of satellite ECEF position expressed in meters (m).
     */
    private double mY;

    /**
     * Z coordinate of satellite ECEF position expressed in meters (m).
     */
    private double mZ;

    /**
     * X coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double mVx;

    /**
     * Y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double mVy;

    /**
     * Z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    private double mVz;

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
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate,
                           final Point3D position,
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
    public GNSSMeasurement(final double pseudoRange, final double pseudoRate,
                           final ECEFPosition position,
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
    public GNSSMeasurement(final Distance pseudoRange, final Speed pseudoRate,
                           final ECEFPosition position,
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
        return mPseudoRange;
    }

    /**
     * Sets pseudo-range measurement expressed in meters (m).
     *
     * @param pseudoRange pseudo-range measurement expressed in meters (m).
     */
    public void setPseudoRange(final double pseudoRange) {
        mPseudoRange = pseudoRange;
    }

    /**
     * Gets pseudo-range measurement.
     *
     * @param result instance where pseudo-range measurement will be stored.
     */
    public void getPseudoRangeDistance(final Distance result) {
        result.setValue(mPseudoRange);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement.
     *
     * @return pseudo-range measurement.
     */
    public Distance getPseudoRangeDistance() {
        return new Distance(mPseudoRange, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement.
     *
     * @param pseudoRange pseudo-range measurement.
     */
    public void setPseudoRangeDistance(Distance pseudoRange) {
        mPseudoRange = DistanceConverter.convert(pseudoRange.getValue().doubleValue(),
                pseudoRange.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement expressed in meters per second (m/s).
     *
     * @return pseudo-range rate measurement expressed in meters per second (m/s).
     */
    public double getPseudoRate() {
        return mPseudoRate;
    }

    /**
     * Sets pseudo-range rate measurement expressed in meters per second (m/s).
     *
     * @param pseudoRate pseudo-range rate measurement expressed in meters per second (m/s).
     */
    public void setPseudoRate(final double pseudoRate) {
        mPseudoRate = pseudoRate;
    }

    /**
     * Gets pseudo-range rate measurement.
     *
     * @param result instance where pseudo-range rate measurement will be stored.
     */
    public void getPseudoRateSpeed(final Speed result) {
        result.setValue(mPseudoRate);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement.
     *
     * @return pseudo-range rate measurement.
     */
    public Speed getPseudoRateSpeed() {
        return new Speed(mPseudoRate, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets pseudo-range rate measurement.
     *
     * @param pseudoRate pseudo-range rate measurement.
     */
    public void setPseudoRateSpeed(final Speed pseudoRate) {
        mPseudoRate = SpeedConverter.convert(pseudoRate.getValue().doubleValue(),
                pseudoRate.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return x coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets x coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param x x coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets x coordinate of satellite ECEF position.
     *
     * @param result instance where x coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of satellite ECEF position.
     *
     * @return x coordinate of satellite ECEF position.
     */
    public Distance getDistanceX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of satellite ECEF position.
     *
     * @param x x coordinate of satellite ECEF position.
     */
    public void setDistanceX(final Distance x) {
        mX = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return y coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets y coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param y y coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets y coordinate of satellite ECEF position.
     *
     * @param result instance where y coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of satellite ECEF position.
     *
     * @return y coordinate of satellite ECEF position.
     */
    public Distance getDistanceY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of satellite ECEF position.
     *
     * @param y y coordinate of satellite ECEF position.
     */
    public void setDistanceY(final Distance y) {
        mY = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of satellite ECEF position expressed in meters (m).
     *
     * @return z coordinate of satellite ECEF position expressed in meters (m).
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets z coordinate of satellite ECEF position expressed in meters (m).
     *
     * @param z z coordinate of satellite ECEF position expressed in meters (m).
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Gets z coordinate of satellite ECEF position.
     *
     * @param result instance where z coordinate of satellite ECEF position will be
     *               stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of satellite ECEF position.
     *
     * @return z coordinate of satellite ECEF position.
     */
    public Distance getDistanceZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of satellite ECEF position.
     *
     * @param z z coordinate of satellite ECEF position.
     */
    public void setDistanceZ(final Distance z) {
        mZ = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Sets coordinates of satellite ECEF position expressed in meters (m).
     *
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
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
    public void getPosition(Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Gets satellite ECEF position expressed in meters (m).
     *
     * @return ECEF position of satellite.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Sets ECEF position of satellite expressed in meters (m).
     *
     * @param position ECEF position of satellite.
     */
    public void setPosition(final Point3D position) {
        mX = position.getInhomX();
        mY = position.getInhomY();
        mZ = position.getInhomZ();
    }

    /**
     * Gets ECEF position of satellite.
     *
     * @param result instance where ECEF position of satellite will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(mX, mY, mZ);
    }

    /**
     * Gets ECEF position of satellite.
     *
     * @return ECEF position of satellite.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(mX, mY, mZ);
    }

    /**
     * Sets ECEF position of satellite.
     *
     * @param ecefPosition ECEF position of satellite.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        mX = ecefPosition.getX();
        mY = ecefPosition.getY();
        mZ = ecefPosition.getZ();
    }

    /**
     * Gets x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets x coordinate of satellite ECEF velocity.
     *
     * @param result instance where x coordinate of satellite ECEF velocity will
     *               be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of satellite ECEF velocity.
     *
     * @return x coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of satellite ECEF velocity.
     *
     * @param speedX x coordinate of satellite ECEF velocity.
     */
    public void setSpeedX(final Speed speedX) {
        mVx = SpeedConverter.convert(speedX.getValue().doubleValue(), speedX.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vy y coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets y coordinate of satellite ECEF velocity.
     *
     * @param result instance where y coordinate of satellite ECEF velocity will
     *               be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of satellite ECEF velocity.
     *
     * @return y coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of satellite ECEF velocity.
     *
     * @param speedY y coordinate of satellite ECEF velocity.
     */
    public void setSpeedY(final Speed speedY) {
        mVy = SpeedConverter.convert(speedY.getValue().doubleValue(), speedY.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @return z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     *
     * @param vz z coordinate of satellite ECEF velocity expressed in meters per second (m/s).
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Gets z coordinate of satellite ECEF velocity.
     *
     * @param result z coordinate of satellite ECEF velocity.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of satellite ECEF velocity.
     *
     * @return z coordinate of satellite ECEF velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of satellite ECEF velocity.
     *
     * @param speedZ z coordinate of satellite ECEF velocity.
     */
    public void setSpeedZ(final Speed speedZ) {
        mVz = SpeedConverter.convert(speedZ.getValue().doubleValue(), speedZ.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets ECEF coordinates of satellite velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of satellite ECEF velocity.
     * @param vy y coordinate of satellite ECEF velocity.
     * @param vz z coordinate of satellite ECEF velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
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
        result.setCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets ECEF velocity of satellite.
     *
     * @return ECEF velocity of satellite.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(mVx, mVy, mVz);
    }

    /**
     * Sets ECEF velocity of satellite.
     *
     * @param ecefVelocity ECEF velocity of satellite.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        mVx = ecefVelocity.getVx();
        mVy = ecefVelocity.getVy();
        mVz = ecefVelocity.getVz();
    }

    /**
     * Gets ECEF position and velocity of satellite.
     *
     * @param result instance where position and velocity will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(mX, mY, mZ);
        result.setVelocityCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets ECEF position and velocity of satellite.
     *
     * @return ECEF position and velocity of satellite.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(mX, mY, mZ, mVx, mVy, mVz);
    }

    /**
     * Sets ECEF position and velocity of satellite.
     *
     * @param positionAndVelocity ECEF position and velocity of satellite.
     */
    public void setPositionAndVelocity(final ECEFPositionAndVelocity positionAndVelocity) {
        mX = positionAndVelocity.getX();
        mY = positionAndVelocity.getY();
        mZ = positionAndVelocity.getZ();

        mVx = positionAndVelocity.getVx();
        mVy = positionAndVelocity.getVy();
        mVz = positionAndVelocity.getVz();
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(GNSSMeasurement output) {
        output.mPseudoRange = mPseudoRange;
        output.mPseudoRate = mPseudoRate;

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
    public void copyFrom(final GNSSMeasurement input) {
        mPseudoRange = input.mPseudoRange;
        mPseudoRate = input.mPseudoRate;

        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mPseudoRange, mPseudoRate, mX, mY, mZ, mVx, mVy, mVz);
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

        final GNSSMeasurement other = (GNSSMeasurement) obj;
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

        return Math.abs(mPseudoRange - other.mPseudoRange) <= threshold
                && Math.abs(mPseudoRate - other.mPseudoRate) <= threshold
                && Math.abs(mX - other.mX) <= threshold
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
        final GNSSMeasurement result = (GNSSMeasurement) super.clone();
        copyTo(result);
        return result;
    }
}
