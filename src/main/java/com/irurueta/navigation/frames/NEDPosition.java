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

import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body curvilinear position with respect Earth, expressed in latitude, longitude and height.
 */
@SuppressWarnings("WeakerAccess")
public class NEDPosition implements Serializable, Cloneable {

    /**
     * Latitude expressed in radians (rad).
     */
    private double mLatitude;

    /**
     * Longitude expressed in radians (rad).
     */
    private double mLongitude;

    /**
     * Height expressed in meters (m).
     */
    private double mHeight;

    /**
     * Constructor.
     */
    public NEDPosition() {
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     */
    public NEDPosition(final double latitude, final double longitude, final double height) {
        setCoordinates(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     */
    public NEDPosition(final Angle latitude, final Angle longitude, final Distance height) {
        setCoordinates(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param input position to copy data from.
     */
    public NEDPosition(final NEDPosition input) {
        copyFrom(input);
    }

    /**
     * Gets latitude expressed in radians (rad).
     *
     * @return latitude expressed in radians.
     */
    public double getLatitude() {
        return mLatitude;
    }

    /**
     * Sets latitude expressed in radians (rad).
     *
     * @param latitude latitude expressed in radians to be set.
     */
    public void setLatitude(final double latitude) {
        mLatitude = latitude;
    }

    /**
     * Gets longitude expressed in radians (rad).
     *
     * @return longitude expressed in radians.
     */
    public double getLongitude() {
        return mLongitude;
    }

    /**
     * Sets longitude expressed in radians (rad).
     *
     * @param longitude longitude expressed in radians to be set.
     */
    public void setLongitude(final double longitude) {
        mLongitude = longitude;
    }

    /**
     * Gets height expressed in meters (m).
     *
     * @return height expressed in meters.
     */
    public double getHeight() {
        return mHeight;
    }

    /**
     * Sets height expressed in meters (m).
     *
     * @param height height expressed in meters to be set.
     */
    public void setHeight(final double height) {
        mHeight = height;
    }

    /**
     * Sets curvilinear position coordinates.
     *
     * @param latitude  latitude expressed in radians (rad) to be set.
     * @param longitude longitude expressed in radians (rad) to be set.
     * @param height    height expressed in meters (m) to be set.
     */
    public void setCoordinates(
            final double latitude, final double longitude, final double height) {
        mLatitude = latitude;
        mLongitude = longitude;
        mHeight = height;
    }

    /**
     * Gets latitude angle.
     *
     * @param result instance where result will be stored.
     */
    public void getLatitudeAngle(final Angle result) {
        result.setValue(mLatitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets latitude angle.
     *
     * @return latitude angle.
     */
    public Angle getLatitudeAngle() {
        return new Angle(mLatitude, AngleUnit.RADIANS);
    }

    /**
     * Sets latitude angle.
     *
     * @param latitude latitude angle to be set.
     */
    public void setLatitudeAngle(final Angle latitude) {
        mLatitude = AngleConverter.convert(latitude.getValue().doubleValue(),
                latitude.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Gets longitude angle.
     *
     * @param result instance where result will be stored.
     */
    public void getLongitudeAngle(final Angle result) {
        result.setValue(mLongitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets longitude angle.
     *
     * @return longitude angle.
     */
    public Angle getLongitudeAngle() {
        return new Angle(mLongitude, AngleUnit.RADIANS);
    }

    /**
     * Sets longitude angle.
     *
     * @param longitude longitude angle to be set.
     */
    public void setLongitudeAngle(final Angle longitude) {
        mLongitude = AngleConverter.convert(longitude.getValue().doubleValue(),
                longitude.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Gets height distance.
     *
     * @param result instance where result will be stored.
     */
    public void getHeightDistance(final Distance result) {
        result.setValue(mHeight);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets height distance.
     *
     * @return height distance.
     */
    public Distance getHeightDistance() {
        return new Distance(mHeight, DistanceUnit.METER);
    }

    /**
     * Sets height distance.
     *
     * @param height height distance to be set.
     */
    public void setHeightDistance(final Distance height) {
        mHeight = DistanceConverter.convert(height.getValue().doubleValue(),
                height.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets curvilinear position coordinates.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     */
    public void setCoordinates(
            final Angle latitude, final Angle longitude, final Distance height) {
        setLatitudeAngle(latitude);
        setLongitudeAngle(longitude);
        setHeightDistance(height);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDPosition output) {
        output.mLatitude = mLatitude;
        output.mLongitude = mLongitude;
        output.mHeight = mHeight;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDPosition input) {
        mLatitude = input.mLatitude;
        mLongitude = input.mLongitude;
        mHeight = input.mHeight;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mLatitude, mLongitude, mHeight);
    }

    /**
     * Checks if provided object is a NEDPosition having exactly the same contents as
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
        if (!(obj instanceof NEDPosition)) {
            return false;
        }

        final NEDPosition other = (NEDPosition) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final NEDPosition other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position coordinates.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final NEDPosition other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mLatitude - other.mLatitude) <= threshold
                && Math.abs(mLongitude - other.mLongitude) <= threshold
                && Math.abs(mHeight - other.mHeight) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final NEDPosition result = (NEDPosition) super.clone();
        copyTo(result);
        return result;
    }
}
