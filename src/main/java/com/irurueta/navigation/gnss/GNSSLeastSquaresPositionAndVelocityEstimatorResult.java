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
 * Contains result of GNSS position and velocity estimation.
 */
public class GNSSLeastSquaresPositionAndVelocityEstimatorResult
        implements Serializable, Cloneable {

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
    private double mEstimatedReceiverClockOffset;

    /**
     * Estimated receiver clock drift expressed in meters per second (m/s).
     */
    private double mEstimatedReceiverClockDrift;

    /**
     * Constructor.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult() {
    }

    /**
     * Constructor.
     *
     * @param x                            x coordinate of estimated ECEF user position
     *                                     expressed in meters (m).
     * @param y                            y coordinate of estimated ECEF user position
     *                                     expressed in meters (m).
     * @param z                            z coordinate of estimated ECEF user position
     *                                     expressed in meters (m).
     * @param vx                           x coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param vy                           y coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param vz                           z coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param estimatedReceiverClockOffset estimated receiver clock offset expressed in
     *                                     meters (m)
     * @param estimatedReceiverClockDrift  estimated receiver clock drift expressed in
     *                                     meters per second (m/s).
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double estimatedReceiverClockOffset,
            final double estimatedReceiverClockDrift) {
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param x                            x coordinate of estimated ECEF user position.
     * @param y                            y coordinate of estimated ECEF user position.
     * @param z                            z coordinate of estimated ECEF user position.
     * @param vx                           x coordinate of estimated ECEF user velocity.
     * @param vy                           y coordinate of estimated ECEF user velocity.
     * @param vz                           z coordinate of estimated ECEF user velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset.
     * @param estimatedReceiverClockDrift  estimated receiver clock drift.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final Distance x, final Distance y, final Distance z,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance estimatedReceiverClockOffset,
            final Speed estimatedReceiverClockDrift) {
        setPositionCoordinates(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param position                     estimated ECEF user position.
     * @param vx                           x coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param vy                           y coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param vz                           z coordinate of estimated ECEF user velocity
     *                                     expressed in meters per second (m/s).
     * @param estimatedReceiverClockOffset estimated receiver clock offset expressed in
     *                                     meters (m)
     * @param estimatedReceiverClockDrift  estimated receiver clock drift expressed in
     *                                     meters per second (m/s).
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final Point3D position,
            final double vx, final double vy, final double vz,
            final double estimatedReceiverClockOffset,
            final double estimatedReceiverClockDrift) {
        setPosition(position);
        setVelocityCoordinates(vx, vy, vz);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param position                     estimated ECEF user position.
     * @param vx                           x coordinate of estimated ECEF user velocity.
     * @param vy                           y coordinate of estimated ECEF user velocity.
     * @param vz                           z coordinate of estimated ECEF user velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset.
     * @param estimatedReceiverClockDrift  estimated receiver clock drift.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final Point3D position,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance estimatedReceiverClockOffset,
            final Speed estimatedReceiverClockDrift) {
        setPosition(position);
        setVelocityCoordinates(vx, vy, vz);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param position                     estimated ECEF user position.
     * @param velocity                     estimated ECEF user velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset expressed in
     *                                     meters (m)
     * @param estimatedReceiverClockDrift  estimated receiver clock drift expressed in
     *                                     meters per second (m/s).
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final ECEFPosition position,
            final ECEFVelocity velocity,
            final double estimatedReceiverClockOffset,
            final double estimatedReceiverClockDrift) {
        setEcefPosition(position);
        setEcefVelocity(velocity);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param position                     estimated ECEF user position.
     * @param velocity                     estimated ECEF user velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset.
     * @param estimatedReceiverClockDrift  estimated receiver clock drift.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final ECEFPosition position,
            final ECEFVelocity velocity,
            final Distance estimatedReceiverClockOffset,
            final Speed estimatedReceiverClockDrift) {
        setEcefPosition(position);
        setEcefVelocity(velocity);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param positionAndVelocity          estimated ECEF user position and velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset expressed in
     *                                     meters (m)
     * @param estimatedReceiverClockDrift  estimated receiver clock drift expressed in
     *                                     meters per second (m/s).
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double estimatedReceiverClockOffset,
            final double estimatedReceiverClockDrift) {
        setPositionAndVelocity(positionAndVelocity);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param positionAndVelocity          estimated ECEF user position and velocity.
     * @param estimatedReceiverClockOffset estimated receiver clock offset.
     * @param estimatedReceiverClockDrift  estimated receiver clock drift.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorResult(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Distance estimatedReceiverClockOffset,
            final Speed estimatedReceiverClockDrift) {
        setPositionAndVelocity(positionAndVelocity);
        setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);
        setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);
    }

    /**
     * Gets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param y y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param z z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets coordinates of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position.
     * @param y y coordinate of estimated ECEF user position.
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Gets x coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @return x coordinate of estimated ECEF user velocity expressed in meters per
     * second (m/s).
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @param vx x coordinate of estimated ECEF user velocity expressed in meters per
     *           second (m/s).
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @return y coordinate of estimated ECEF user velocity expressed in meters per
     * second (m/s).
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @param vy y coordinate of estimated ECEF user velocity expressed in meters per
     *           second (m/s).
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @return z coordinate of estimated ECEF user velocity expressed in meters per
     * second (m/s).
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @param vz z coordinate of estimated ECEF user velocity expressed in meters per
     *           second (m/s).
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets coordinates of estimated ECEF user velocity expressed in meters per second
     * (m/s).
     *
     * @param vx x coordinate of estimated ECEF user velocity.
     * @param vy y coordinate of estimated ECEF user velocity.
     * @param vz z coordinate of estimated ECEF user velocity.
     */
    public void setVelocityCoordinates(
            final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets estimated receiver clock offset expressed in meters (m).
     *
     * @return estimated receiver clock offset expressed in meters (m).
     */
    public double getEstimatedReceiverClockOffset() {
        return mEstimatedReceiverClockOffset;
    }

    /**
     * Sets estimated receiver clock offset expressed in meters (m).
     *
     * @param estimatedReceiverClockOffset estimated receiver clock offset expressed
     *                                     in meters (m).
     */
    public void setEstimatedReceiverClockOffset(
            final double estimatedReceiverClockOffset) {
        mEstimatedReceiverClockOffset = estimatedReceiverClockOffset;
    }

    /**
     * Gets estimated receiver clock drift expressed in meters per second (m/s).
     *
     * @return estimated receiver clock drift expressed in meters per second (m/s).
     */
    public double getEstimatedReceiverClockDrift() {
        return mEstimatedReceiverClockDrift;
    }

    /**
     * Sets estimated receiver clock drift expressed in meters per second (m/s).
     *
     * @param estimatedReceiverClockDrift estimated receiver clock drift expressed
     *                                    in meters per second (m/s).
     */
    public void setEstimatedReceiverClockDrift(
            final double estimatedReceiverClockDrift) {
        mEstimatedReceiverClockDrift = estimatedReceiverClockDrift;
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @param result instance where result will be stored.
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
        mX = DistanceConverter.convert(x.getValue().doubleValue(),
                x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @param result instance where result will be stored.
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
        mY = DistanceConverter.convert(y.getValue().doubleValue(),
                y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @param result instance where result will be stored.
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
        mZ = DistanceConverter.convert(z.getValue().doubleValue(),
                z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets coordinates of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     * @param y y coordinate of estimated ECEF user position.
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setPositionCoordinates(final Distance x, final Distance y,
                                       final Distance z) {
        setDistanceX(x);
        setDistanceY(y);
        setDistanceZ(z);
    }

    /**
     * Gets x coordinate of estimated ECEF user velocity.
     *
     * @param result instance where result will be stored.
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
     * @param vx x coordinate of estimated ECEF user velocity.
     */
    public void setSpeedX(final Speed vx) {
        mVx = SpeedConverter.convert(vx.getValue().doubleValue(),
                vx.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of estimated ECEF user velocity.
     *
     * @param result instance where result will be stored.
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
     * @param vy y coordinate of estimated ECEF user velocity.
     */
    public void setSpeedY(final Speed vy) {
        mVy = SpeedConverter.convert(vy.getValue().doubleValue(),
                vy.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of estimated ECEF user velocity.
     *
     * @param result instance where result will be stored.
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
     * @param vz z coordinate of estimated ECEF user velocity.
     */
    public void setSpeedZ(final Speed vz) {
        mVz = SpeedConverter.convert(vz.getValue().doubleValue(),
                vz.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinates of estimated ECEF user velocity.
     *
     * @param vx x coordinate of estimated ECEF user velocity.
     * @param vy y coordinate of estimated ECEF user velocity.
     * @param vz z coordinate of estimated ECEF user velocity.
     */
    public void setVelocityCoordinates(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedX(vx);
        setSpeedY(vy);
        setSpeedZ(vz);
    }

    /**
     * Gets estimated receiver clock offset.
     *
     * @param result instance where result will be stored.
     */
    public void getEstimatedReceiverClockOffsetDistance(final Distance result) {
        result.setValue(mEstimatedReceiverClockOffset);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock offset.
     *
     * @return estimated receiver clock offset.
     */
    public Distance getEstimatedReceiverClockOffsetDistance() {
        return new Distance(mEstimatedReceiverClockOffset,
                DistanceUnit.METER);
    }

    /**
     * Sets estimated receiver clock offset.
     *
     * @param estimatedReceiverClockOffset estimated receiver clock offset.
     */
    public void setEstimatedReceiverClockOffset(
            final Distance estimatedReceiverClockOffset) {
        mEstimatedReceiverClockOffset = DistanceConverter.convert(
                estimatedReceiverClockOffset.getValue().doubleValue(),
                estimatedReceiverClockOffset.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock drift.
     *
     * @param result instance where result will be stored.
     */
    public void getEstimatedReceiverClockDriftSpeed(final Speed result) {
        result.setValue(mEstimatedReceiverClockDrift);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated receiver clock drift.
     *
     * @return estimated receiver clock drift.
     */
    public Speed getEstimatedReceiverClockDriftSpeed() {
        return new Speed(mEstimatedReceiverClockDrift,
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated receiver clock drift.
     *
     * @param estimatedReceiverClockDrift estimated receiver clock drift.
     */
    public void setEstimatedReceiverClockDrift(final Speed estimatedReceiverClockDrift) {
        mEstimatedReceiverClockDrift = SpeedConverter.convert(
                estimatedReceiverClockDrift.getValue().doubleValue(),
                estimatedReceiverClockDrift.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @param result instance where result will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @return estimated ECEF user position expressed in meters (m).
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Sets estimated ECEF user position expressed in meters (m).
     *
     * @param position estimated ECEF user position expressed in meters (m).
     */
    public void setPosition(final Point3D position) {
        mX = position.getInhomX();
        mY = position.getInhomY();
        mZ = position.getInhomZ();
    }

    /**
     * Gets estimated ECEF user position.
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
        setPositionCoordinates(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ());
        setVelocityCoordinates(positionAndVelocity.getVx(), positionAndVelocity.getVy(),
                positionAndVelocity.getVz());
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(GNSSLeastSquaresPositionAndVelocityEstimatorResult output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;

        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;

        output.mEstimatedReceiverClockOffset = mEstimatedReceiverClockOffset;
        output.mEstimatedReceiverClockDrift = mEstimatedReceiverClockDrift;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(GNSSLeastSquaresPositionAndVelocityEstimatorResult input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;

        mEstimatedReceiverClockOffset = input.mEstimatedReceiverClockOffset;
        mEstimatedReceiverClockDrift = input.mEstimatedReceiverClockDrift;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mX, mY, mZ, mVx, mVy, mVz,
                mEstimatedReceiverClockOffset, mEstimatedReceiverClockDrift);
    }

    /**
     * Checks if provided object is a GNSSLeastSquaresPositionAndVelocityEstimatorResult
     * having exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        GNSSLeastSquaresPositionAndVelocityEstimatorResult other =
                (GNSSLeastSquaresPositionAndVelocityEstimatorResult) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(
            final GNSSLeastSquaresPositionAndVelocityEstimatorResult other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(
            final GNSSLeastSquaresPositionAndVelocityEstimatorResult other,
            final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold
                && Math.abs(mEstimatedReceiverClockOffset - other.mEstimatedReceiverClockOffset) <= threshold
                && Math.abs(mEstimatedReceiverClockDrift - other.mEstimatedReceiverClockDrift) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                (GNSSLeastSquaresPositionAndVelocityEstimatorResult) super.clone();
        copyTo(result);
        return result;
    }
}
