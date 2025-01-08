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
    private double x;

    /**
     * Y coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double y;

    /**
     * Z coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double z;

    /**
     * X coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double vx;

    /**
     * Y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double vy;

    /**
     * Z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     */
    private double vz;

    /**
     * Estimated receiver clock offset expressed in meters (m).
     */
    private double clockOffset;

    /**
     * Estimated receiver clock drift expressed in meters per second (m/s).
     */
    private double clockDrift;

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
    public GNSSEstimation(final Point3D position, final double vx, final double vy, final double vz,
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
    public GNSSEstimation(final Point3D position, final Speed vx, final Speed vy, final Speed vz,
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
    public GNSSEstimation(final ECEFPosition position, final ECEFVelocity velocity,
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
    public GNSSEstimation(final ECEFPosition position, final ECEFVelocity velocity, final Distance clockOffset,
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
    public GNSSEstimation(final ECEFPositionAndVelocity positionAndVelocity, final double clockOffset,
                          final double clockDrift) {
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
    public GNSSEstimation(final ECEFPositionAndVelocity positionAndVelocity, final Distance clockOffset,
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
        return x;
    }

    /**
     * Sets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public double getY() {
        return y;
    }

    /**
     * Sets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setZ(final double z) {
        this.z = z;
    }

    /**
     * Sets coordinates of estimated ECEF user position expressed in meters (m).
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
     * Gets x coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return x coordinate of estimated ECEF user velocity.
     */
    public double getVx() {
        return vx;
    }

    /**
     * Sets x coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate of estimated ECEF user velocity.
     */
    public void setVx(final double vx) {
        this.vx = vx;
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return y coordinate of estimated ECEF user velocity.
     */
    public double getVy() {
        return vy;
    }

    /**
     * Sets y coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vy y coordinate of estimated ECEF user velocity.
     */
    public void setVy(final double vy) {
        this.vy = vy;
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @return z coordinate of estimated ECEF user velocity.
     */
    public double getVz() {
        return vz;
    }

    /**
     * Sets z coordinate of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vz z coordinate of estimated ECEF user velocity.
     */
    public void setVz(final double vz) {
        this.vz = vz;
    }

    /**
     * Sets coordinates of estimated ECEF user velocity expressed in meters per second (m/s).
     *
     * @param vx x coordinate.
     * @param vy y coordinate.
     * @param vz z coordinate.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    /**
     * Gets estimated receiver clock offset expressed in meters (m).
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * will ultimately be propagated as distance errors.
     *
     * @return estimated receiver clock offset.
     */
    public double getClockOffset() {
        return clockOffset;
    }

    /**
     * Sets estimated receiver clock offset expressed in meters (m).
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * are ultimately propagated as distance errors.
     *
     * @param clockOffset estimated receiver clock offset.
     */
    public void setClockOffset(final double clockOffset) {
        this.clockOffset = clockOffset;
    }

    /**
     * Gets estimated receiver clock drift expressed in meters per second (m/s).
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @return estimated receiver clock drift.
     */
    public double getClockDrift() {
        return clockDrift;
    }

    /**
     * Sets estimated receiver clock drift expressed in meters per second (m/s).
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param clockDrift estimated receiver clock drift.
     */
    public void setClockDrift(final double clockDrift) {
        this.clockDrift = clockDrift;
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @param result instance where x coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(x);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @return x coordinate of estimated ECEF user position.
     */
    public Distance getDistanceX() {
        return new Distance(x, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setDistanceX(final Distance x) {
        this.x = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @param result instance where y coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(y);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public Distance getDistanceY() {
        return new Distance(y, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of estimated ECEF user position.
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setDistanceY(final Distance y) {
        this.y = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @param result instance where z coordinate of estimated ECEF user position will be stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(z);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public Distance getDistanceZ() {
        return new Distance(z, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of estimated ECEF user position.
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setDistanceZ(final Distance z) {
        this.z = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(), DistanceUnit.METER);
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
        result.setValue(vx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of estimated ECEF user velocity.
     *
     * @return x coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedX() {
        return new Speed(vx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of estimated ECEF user velocity.
     *
     * @param speedX x coordinate of estimated ECEF user velocity.
     */
    public void setSpeedX(final Speed speedX) {
        vx = SpeedConverter.convert(speedX.getValue().doubleValue(), speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity.
     *
     * @param result instance where y coordinate of estimated ECEF user velocity will
     *               be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(vy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity.
     *
     * @return y coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedY() {
        return new Speed(vy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of estimated ECEF user velocity.
     *
     * @param speedY y coordinate of estimated ECEF user velocity.
     */
    public void setSpeedY(final Speed speedY) {
        vy = SpeedConverter.convert(speedY.getValue().doubleValue(), speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity.
     *
     * @param result instance where z coordinate of estimated ECEF user velocity will
     *               be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(vz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity.
     *
     * @return z coordinate of estimated ECEF user velocity.
     */
    public Speed getSpeedZ() {
        return new Speed(vz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of estimated ECEF user velocity.
     *
     * @param speedZ z coordinate of estimated ECEF user velocity.
     */
    public void setSpeedZ(final Speed speedZ) {
        vz = SpeedConverter.convert(speedZ.getValue().doubleValue(), speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);
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
        result.setValue(clockOffset);
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
        return new Distance(clockOffset, DistanceUnit.METER);
    }

    /**
     * Sets estimated receiver clock offset.
     * Notice that clock offset is estimated in terms of distance, since timing errors
     * are ultimately propagated as distance errors.
     *
     * @param clockOffset estimated receiver clock offset.
     */
    public void setClockOffset(final Distance clockOffset) {
        this.clockOffset = DistanceConverter.convert(clockOffset.getValue().doubleValue(), clockOffset.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock drift.
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param result instance where estimated receiver clock drift will be stored.
     */
    public void getClockDriftSpeed(final Speed result) {
        result.setValue(clockDrift);
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
        return new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated receiver clock drift.
     * Notice that the rate at which clock errors increase or decrease will ultimately
     * propagate as speed (and hence position) errors.
     *
     * @param clockDrift estimated receiver clock drift.
     */
    public void setClockDrift(final Speed clockDrift) {
        this.clockDrift = SpeedConverter.convert(clockDrift.getValue().doubleValue(), clockDrift.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @param result instance where estimated ECEF user position will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(x, y, z);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @return estimated ECEF user position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(x, y, z);
    }

    /**
     * Sets estimated ECEF user position expressed in meters (m).
     *
     * @param position estimated ECEF user position.
     */
    public void setPosition(final Point3D position) {
        x = position.getInhomX();
        y = position.getInhomY();
        z = position.getInhomZ();
    }

    /**
     * Gets estimatedECEF user position.
     *
     * @param result instance where result will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(x, y, z);
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @return estimated ECEF user position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(x, y, z);
    }

    /**
     * Sets estimated ECEF user position.
     *
     * @param ecefPosition estimated ECEF user position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        x = ecefPosition.getX();
        y = ecefPosition.getY();
        z = ecefPosition.getZ();
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @param result instance where result will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(vx, vy, vz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @return estimated ECEF user velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(vx, vy, vz);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param ecefVelocity estimated ECEF user velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        vx = ecefVelocity.getVx();
        vy = ecefVelocity.getVy();
        vz = ecefVelocity.getVz();
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(x, y, z);
        result.setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @return estimated ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
    }

    /**
     * Sets estimated ECEF user position and velocity.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     */
    public void setPositionAndVelocity(final ECEFPositionAndVelocity positionAndVelocity) {
        setPositionCoordinates(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ());
        setVelocityCoordinates(positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz());
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

        result[0] = x;
        result[1] = y;
        result[2] = z;
        result[3] = vx;
        result[4] = vy;
        result[5] = vz;
        result[6] = clockOffset;
        result[7] = clockDrift;
    }

    /**
     * Converts state data into an array.
     *
     * @return a new array containing state data.
     */
    public double[] asArray() {
        final var result = new double[NUM_PARAMETERS];
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

        x = array[0];
        y = array[1];
        z = array[2];
        vx = array[3];
        vy = array[4];
        vz = array[5];
        clockOffset = array[6];
        clockDrift = array[7];
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

        result.setElementAtIndex(0, x);
        result.setElementAtIndex(1, y);
        result.setElementAtIndex(2, z);
        result.setElementAtIndex(3, vx);
        result.setElementAtIndex(4, vy);
        result.setElementAtIndex(5, vz);
        result.setElementAtIndex(6, clockOffset);
        result.setElementAtIndex(7, clockDrift);
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
        output.x = x;
        output.y = y;
        output.z = z;

        output.vx = vx;
        output.vy = vy;
        output.vz = vz;

        output.clockOffset = clockOffset;
        output.clockDrift = clockDrift;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSEstimation input) {
        x = input.x;
        y = input.y;
        z = input.z;

        vx = input.vx;
        vy = input.vy;
        vz = input.vz;

        clockOffset = input.clockOffset;
        clockDrift = input.clockDrift;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(x, y, z, vx, vy, vz, clockOffset, clockDrift);
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

        return Math.abs(x - other.x) <= threshold
                && Math.abs(y - other.y) <= threshold
                && Math.abs(z - other.z) <= threshold
                && Math.abs(vx - other.vx) <= threshold
                && Math.abs(vy - other.vy) <= threshold
                && Math.abs(vz - other.vz) <= threshold
                && Math.abs(clockOffset - other.clockOffset) <= threshold
                && Math.abs(clockDrift - other.clockDrift) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (GNSSEstimation) super.clone();
        copyTo(result);
        return result;
    }
}
