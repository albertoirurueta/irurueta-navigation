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

import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body velocity with respect Earth, resolved about north, east and down an expressed in meters per second
 * (m/s).
 */
public class NEDVelocity implements Serializable, Cloneable {
    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     */
    private double vn;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     */
    private double ve;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     */
    private double vd;

    /**
     * Constructor.
     */
    public NEDVelocity() {
    }

    /**
     * Constructor.
     *
     * @param vn North velocity coordinate value expressed in meters per second (m/s).
     * @param ve East velocity coordinate value expressed in meters per second (m/s).
     * @param vd Down velocity coordinate value expressed in meters per second (m/s).
     */
    public NEDVelocity(final double vn, final double ve, final double vd) {
        setCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param vn North velocity coordinate value.
     * @param ve East velocity coordinate value.
     * @param vd Down velocity coordinate value.
     */
    public NEDVelocity(final Speed vn, final Speed ve, final Speed vd) {
        setCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param input Body velocity to copy data from.
     */
    public NEDVelocity(final NEDVelocity input) {
        copyFrom(input);
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate value.
     */
    public double getVn() {
        return vn;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @param vn North velocity coordinate value.
     */
    public void setVn(final double vn) {
        this.vn = vn;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate value.
     */
    public double getVe() {
        return ve;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @param ve East velocity coordinate value.
     */
    public void setVe(final double ve) {
        this.ve = ve;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate value.
     */
    public double getVd() {
        return vd;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param vd Down velocity coordinate value.
     */
    public void setVd(final double vd) {
        this.vd = vd;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s) resolved along North, East, Down
     * axes.
     *
     * @param vn North velocity coordinate value.
     * @param ve East velocity coordinate value.
     * @param vd Down velocity coordinate value.
     */
    public void setCoordinates(final double vn, final double ve, final double vd) {
        this.vn = vn;
        this.ve = ve;
        this.vd = vd;
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getNorm() {
        return Math.sqrt(vn * vn + ve * ve + vd * vd);
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
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @param result instance where North velocity coordinate will be stored.
     */
    public void getSpeedN(final Speed result) {
        result.setValue(vn);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate.
     */
    public Speed getSpeedN() {
        return new Speed(vn, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @param speedN North velocity coordinate to be set.
     */
    public void setSpeedN(final Speed speedN) {
        vn = SpeedConverter.convert(speedN.getValue().doubleValue(), speedN.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param result instance where East velocity coordinate will be stored.
     */
    public void getSpeedE(final Speed result) {
        result.setValue(ve);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate.
     */
    public Speed getSpeedE() {
        return new Speed(ve, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param speedE East velocity coordinate to be set.
     */
    public void setSpeedE(final Speed speedE) {
        ve = SpeedConverter.convert(speedE.getValue().doubleValue(), speedE.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param result instance where Down velocity coordinate will be stored.
     */
    public void getSpeedD(final Speed result) {
        result.setValue(vd);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate.
     */
    public Speed getSpeedD() {
        return new Speed(vd, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param speedD Down velocity coordinate to be set.
     */
    public void setSpeedD(final Speed speedD) {
        vd = SpeedConverter.convert(speedD.getValue().doubleValue(), speedD.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity coordinates of body frame resolved along North, East, Down
     * axes.
     *
     * @param speedN North velocity coordinate.
     * @param speedE East velocity coordinate.
     * @param speedD Down velocity coordinate.
     */
    public void setCoordinates(final Speed speedN, final Speed speedE, final Speed speedD) {
        setSpeedN(speedN);
        setSpeedE(speedE);
        setSpeedD(speedD);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDVelocity output) {
        output.vn = vn;
        output.ve = ve;
        output.vd = vd;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDVelocity input) {
        vn = input.vn;
        ve = input.ve;
        vd = input.vd;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(vn, ve, vd);
    }

    /**
     * Checks if provided object is a BodyVelocity having exactly the same contents as
     * this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof NEDVelocity other)) {
            return false;
        }

        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final NEDVelocity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between velocity coordinates.
     * @return true if both instances are considered to be equal (up to provided threshold), false otherwise.
     */
    public boolean equals(final NEDVelocity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(vn - other.vn) <= threshold
                && Math.abs(ve - other.ve) <= threshold
                && Math.abs(vd - other.vd) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (NEDVelocity) super.clone();
        copyTo(result);
        return result;
    }
}
