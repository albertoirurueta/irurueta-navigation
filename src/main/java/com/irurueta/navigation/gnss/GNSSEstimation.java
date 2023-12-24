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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
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
 * Contains GNSS state estimation, which contains user
 * position, velocity and estimated clock offset and drift.
 */
public class GNSSEstimation implements Serializable, Cloneable {

    /**
     * Number of parameters stored into Kalman filter state.
     */
    public static final int NUM_PARAMETERS = 8;

    /**
     * X coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mX;

    /**
     * Y coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mY;

    /**
     * Z coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mZ;

    /**
     * X coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double mVx;

    /**
     * Y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double mVy;

    /**
     * Z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double mVz;

    /**
     * Estimated receiver clock offset expressed in meters (m).
     */
    private double mClockOffset;

    /**
     * Estimated receiver clock drift expressed in meters per second (m/s).
     */
    private double mClockDrift;

    /**
     * Constructor.
     */
    public GNSSEstimation() {
    }

    /**
     * Constructor.
     *
     * @param x           x coordinate of estimated ECEF user position expressed in meters (m).
     * @param y           y coordinate of estimated ECEF user position expressed in meters (m).
     * @param z           z coordinate of estimated ECEF user position expressed in meters (m).
     * @param vx          x coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     * @param vy          y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     * @param vz          z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     * @param clockOffset estimated receiver clock offset expressed in meters (m).
     * @param clockDrift  estimated receiver clock drift expressed in meters per second (m/s).
     */
    public GNSSEstimation(final double x, final double y, final double z,
                          final double vx, final double vy, final double vz,
                          final double clockOffset, final double clockDrift) {
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param x           x coordinate of estimated ECEF user position.
     * @param y           y coordinate of estimated ECEF user position.
     * @param z           z coordinate of estimated ECEF user position.
     * @param vx          x coordinate of estimated ECEF user velocity.
     * @param vy          y coordinate of estimated ECEF user velocity.
     * @param vz          z coordinate of estimated ECEF user velocity.
     * @param clockOffset estimated receiver clock offset.
     * @param clockDrift  estimated receiver clock drift.
     */
    public GNSSEstimation(final Distance x, final Distance y, final Distance z,
                          final Speed vx, final Speed vy, final Speed vz,
                          final Distance clockOffset, final Speed clockDrift) {
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param position    estimated ECEF user position.
     * @param vx          x coordinate of estimated ECEF user velocity
     *                    expressed in meters per second (m/s).
     * @param vy          y coordinate of estimated ECEF user velocity
     *                    expressed in meters per second (m/s).
     * @param vz          z coordinate of estimated ECEF user velocity
     *                    expressed in meters per second (m/s).
     * @param clockOffset estimated receiver clock offset expressed in
     *                    meters (m).
     * @param clockDrift  estimated receiver clock drift expressed in
     *                    meters per second (m/s).
     */
    public GNSSEstimation(
            final Point3D position,
            final double vx, final double vy, final double vz,
            final double clockOffset, final double clockDrift) {
        setPosition(position);
        setVelocityCoordinates(vx, vy, vz);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param position    estimated ECEF user position.
     * @param vx          x coordinate of estimated ECEF user velocity.
     * @param vy          y coordinate of estimated ECEF user velocity.
     * @param vz          z coordinate of estimated ECEF user velocity.
     * @param clockOffset estimated receiver clock offset.
     * @param clockDrift  estimated receiver clock drift.
     *                    8
     */
    public GNSSEstimation(final Point3D position,
                          final Speed vx, final Speed vy, final Speed vz,
                          final Distance clockOffset, final Speed clockDrift) {
        setPosition(position);
        setVelocityCoordinates(vx, vy, vz);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param position    estimated ECEF user position.
     * @param velocity    estimated ECEF user velocity.
     * @param clockOffset estimated receiver clock offset expressed in meters (m).
     * @param clockDrift  estimated receiver clock drift expressed in meters per
     *                    second (m/s).
     */
    public GNSSEstimation(
            final ECEFPosition position,
            final ECEFVelocity velocity,
            final double clockOffset, final double clockDrift) {
        setEcefPosition(position);
        setEcefVelocity(velocity);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param position    estimated ECEF user position.
     * @param velocity    estimated ECEF user velocity.
     * @param clockOffset estimated receiver clock offset.
     * @param clockDrift  estimated receiver clock drift.
     */
    public GNSSEstimation(
            final ECEFPosition position,
            final ECEFVelocity velocity,
            final Distance clockOffset,
            final Speed clockDrift) {
        setEcefPosition(position);
        setEcefVelocity(velocity);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     * @param clockOffset         estimated receiver clock offset expressed in
     *                            meters (m).
     * @param clockDrift          estimated receiver clock drift expressed in
     *                            meters per second (m/s).
     */
    public GNSSEstimation(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double clockOffset, final double clockDrift) {
        setPositionAndVelocity(positionAndVelocity);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Constructor.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     * @param clockOffset         estimated receiver clock offset.
     * @param clockDrift          estimated receiver clock drift.
     */
    public GNSSEstimation(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Distance clockOffset,
            final Speed clockDrift) {
        setPositionAndVelocity(positionAndVelocity);
        setClockOffset(clockOffset);
        setClockDrift(clockDrift);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public GNSSEstimation(final GNSSEstimation input) {
        copyFrom(input);
    }

    /**
     * Gets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return x coordinate of estimated ECEF user position.
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets coordinates of estimated ECEF user position expressed in meters (m).
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
     * Gets x coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return x coordinate of estimated ECEF user velocity.
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of estimated ECEF user velocity.
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return y coordinate of estimated ECEF user velocity.
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vy y coordinate of estimated ECEF user velocity.
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return z coordinate of estimated ECEF user velocity.
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vz z coordinate of estimated ECEF user velocity.
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets coordinates of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate.
     * @param vy y coordinate.
     * @param vz z coordinate.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets estimated receiver clock offset expressed in meters (m).
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * will ultimately be propagated as distance errors.
     *
     * @return estimated receiver clock offset.
     */
    public double getClockOffset() {
        return mClockOffset;
    }

    /**
     * Sets estimated receiver clock offset expressed in meters (m).
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * are ultimately propagated as distance errors.
     *
     * @param clockOffset estimated receiver clock offset.
     */
    public void setClockOffset(final double clockOffset) {
        mClockOffset = clockOffset;
    }

    /**
     * Gets estimated receiver clock drift expressed in meters per second (m/s).
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @return estimated receiver clock drift.
     */
    public double getClockDrift() {
        return mClockDrift;
    }

    /**
     * Sets estimated receiver clock drift expressed in meters per second (m/s).
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param clockDrift estimated receiver clock drift.
     */
    public void setClockDrift(final double clockDrift) {
        mClockDrift = clockDrift;
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @param result instance where x coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @return x coordinate of estimated ECEF user position.
     */
    public Distance getDistanceX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setDistanceX(final Distance x) {
        mX = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @param result instance where y coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public Distance getDistanceY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of estimated ECEF user position.
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setDistanceY(final Distance y) {
        mY = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @param result instance where z coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public Distance getDistanceZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of estimated ECEF user position.
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setDistanceZ(final Distance z) {
        mZ = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Sets coordinates of estimated ECEF user position.
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
     * Gets x coordinate of estimated ECEF user velocity.
     *
     * @param result instance where x coordinate of estimated ECEF user velocity will
     *               be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of estimated ECEF user velocity.
     *
     * @return x coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of estimated ECEF user velocity.
     *
     * @param speedX x coordinate of estimated ECEF user velocity.
     */
    public void setSpeedX(final Speed speedX) {
        mVx = SpeedConverter.convert(speedX.getValue().doubleValue(), speedX.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity.
     *
     * @param result instance where y coordinate of estimated ECEF user velocity will
     *               be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity.
     *
     * @return y coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of estimated ECEF user velocity.
     *
     * @param speedY y coordinate of estimated ECEF user velocity.
     */
    public void setSpeedY(final Speed speedY) {
        mVy = SpeedConverter.convert(speedY.getValue().doubleValue(), speedY.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity.
     *
     * @param result instance where z coordinate of estimated ECEF user velocity will
     *               be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity.
     *
     * @return z coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of estimated ECEF user velocity.
     *
     * @param speedZ z coordinate of estimated ECEF user velocity.
     */
    public void setSpeedZ(final Speed speedZ) {
        mVz = SpeedConverter.convert(speedZ.getValue().doubleValue(), speedZ.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinates of estimated ECEF user velocity.
     *
     * @param speedX x coordinate.
     * @param speedY y coordinate.
     * @param speedZ z coordinate.
     */
    public void setVelocityCoordinates(final Speed speedX, final Speed speedY, final Speed speedZ) {
        setSpeedX(speedX);
        setSpeedY(speedY);
        setSpeedZ(speedZ);
    }

    /**
     * Gets estimated receiver clock offset.
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * will ultimately be propagated as distance errors.
     *
     * @param result instance where estimated receiver clock offset will be stored.
     */
    public void getClockOffsetDistance(final Distance result) {
        result.setValue(mClockOffset);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock offset.
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * will ultimately be propagated as distance errors.
     *
     * @return estimated receiver clock offset.
     */
    public Distance getClockOffsetDistance() {
        return new Distance(mClockOffset, DistanceUnit.METER);
    }

    /**
     * Sets estimated receiver clock offset.
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * are ultimately propagated as distance errors.
     *
     * @param clockOffset estimated receiver clock offset.
     */
    public void setClockOffset(final Distance clockOffset) {
        mClockOffset = DistanceConverter.convert(clockOffset.getValue().doubleValue(),
                clockOffset.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock drift.
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param result instance where estimated receiver clock drift will be stored.
     */
    public void getClockDriftSpeed(final Speed result) {
        result.setValue(mClockDrift);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated receiver clock drift.
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @return estimated receiver clock drift.
     */
    public Speed getClockDriftSpeed() {
        return new Speed(mClockDrift, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated receiver clock drift.
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param clockDrift estimated receiver clock drift.
     */
    public void setClockDrift(final Speed clockDrift) {
        mClockDrift = SpeedConverter.convert(clockDrift.getValue().doubleValue(),
                clockDrift.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @param result instance where estimated ECEF user position will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @return estimated ECEF user position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Sets estimated ECEF user position expressed in meters (m).
     *
     * @param position estimated ECEF user position.
     */
    public void setPosition(final Point3D position) {
        mX = position.getInhomX();
        mY = position.getInhomY();
        mZ = position.getInhomZ();
    }

    /**
     * Gets estimatedECEF user position.
     *
     * @param result instance where result will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(mX, mY, mZ);
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @return estimated ECEF user position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(mX, mY, mZ);
    }

    /**
     * Sets estimated ECEF user position.
     *
     * @param ecefPosition estimated ECEF user position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        mX = ecefPosition.getX();
        mY = ecefPosition.getY();
        mZ = ecefPosition.getZ();
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @param result instance where result will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @return estimated ECEF user velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(mVx, mVy, mVz);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param ecefVelocity estimated ECEF user velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        mVx = ecefVelocity.getVx();
        mVy = ecefVelocity.getVy();
        mVz = ecefVelocity.getVz();
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(mX, mY, mZ);
        result.setVelocityCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @return estimated ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(mX, mY, mZ, mVx, mVy, mVz);
    }

    /**
     * Sets estimated ECEF user position and velocity.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     */
    public void setPositionAndVelocity(
            final ECEFPositionAndVelocity positionAndVelocity) {
        setPositionCoordinates(positionAndVelocity.getX(),
                positionAndVelocity.getY(), positionAndVelocity.getZ());
        setVelocityCoordinates(positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz());
    }

    /**
     * Converts state data into an array.
     *
     * @param result instance where state data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 8.
     */
    public void asArray(final double[] result) {
        if (result.length != NUM_PARAMETERS) {
            throw new IllegalArgumentException();
        }

        result[0] = mX;
        result[1] = mY;
        result[2] = mZ;
        result[3] = mVx;
        result[4] = mVy;
        result[5] = mVz;
        result[6] = mClockOffset;
        result[7] = mClockDrift;
    }

    /**
     * Converts state data into an array.
     *
     * @return a new array containing state data.
     */
    public double[] asArray() {
        final double[] result = new double[NUM_PARAMETERS];
        asArray(result);
        return result;
    }

    /**
     * Sets array values into this instance state.
     *
     * @param array array to copy data from.
     * @throws IllegalArgumentException if provided array does not have length 8.
     */
    public void fromArray(final double[] array) {
        if (array.length != NUM_PARAMETERS) {
            throw new IllegalArgumentException();
        }

        mX = array[0];
        mY = array[1];
        mZ = array[2];
        mVx = array[3];
        mVy = array[4];
        mVz = array[5];
        mClockOffset = array[6];
        mClockDrift = array[7];
    }

    /**
     * Converts state data into a column matrix.
     * If provided matrix is not 8x1 it will be resized.
     *
     * @param result instance where state data will be stored.
     */
    public void asMatrix(final Matrix result) {
        if (result.getRows() != NUM_PARAMETERS || result.getColumns() != 1) {
            try {
                result.resize(NUM_PARAMETERS, 1);
            } catch (WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mX);
        result.setElementAtIndex(1, mY);
        result.setElementAtIndex(2, mZ);
        result.setElementAtIndex(3, mVx);
        result.setElementAtIndex(4, mVy);
        result.setElementAtIndex(5, mVz);
        result.setElementAtIndex(6, mClockOffset);
        result.setElementAtIndex(7, mClockDrift);
    }

    /**
     * Converts state data into a column matrix.
     *
     * @return a new 8x1 column matrix containing state data.
     */
    public Matrix asMatrix() {
        final Matrix result;
        try {
            result = new Matrix(NUM_PARAMETERS, 1);
            asMatrix(result);
            return result;
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    /**
     * Sets matrix values into this instance state.
     *
     * @param matrix matrix to copy data from.
     * @throws IllegalArgumentException if provided matrix is not 8x1.
     */
    public void fromMatrix(final Matrix matrix) {
        if (matrix.getRows() != NUM_PARAMETERS || matrix.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        fromArray(matrix.getBuffer());
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSEstimation output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;

        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;

        output.mClockOffset = mClockOffset;
        output.mClockDrift = mClockDrift;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSEstimation input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;

        mClockOffset = input.mClockOffset;
        mClockDrift = input.mClockDrift;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mX, mY, mZ, mVx, mVy, mVz, mClockOffset, mClockDrift);
    }

    /**
     * Checks if provided object is a GNSSKalmanStateEstimates having exactly the same
     * contents as this instance.
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

        final GNSSEstimation other = (GNSSEstimation) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final GNSSEstimation other) {
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
    public boolean equals(final GNSSEstimation other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold
                && Math.abs(mClockOffset - other.mClockOffset) <= threshold
                && Math.abs(mClockDrift - other.mClockDrift) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final GNSSEstimation result = (GNSSEstimation) super.clone();
        copyTo(result);
        return result;
    }
}
