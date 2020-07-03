/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

/**
 * Contains a collection of items containing body kinematics
 * measurements ordered by the timestamp when the measurement was made.
 * Measurements within a sequence will be made while the device is
 * being moved.
 * Samples between sequences will be ignored because it will be assumed
 * that the device will be static.
 * Hence, during static periods, only the mean accelerations will be measured.
 * The mean accelerations during static periods will approximately match the
 * gravity versor expressed in body coordinates.
 *
 * @param <T> a type of {@link TimedBodyKinematics}.
 */
public class BodyKinematicsSequence<T extends TimedBodyKinematics>
        implements Serializable, Cloneable {

    /**
     * List of items.
     * If items are provided unsorted, they are reordered by timestamp on
     * getter method.
     */
    private List<T> mItems;

    /**
     * Contains sorted list of items.
     * This list is kept for performance reasons to reduce the amount of
     * required sorting.
     */
    private List<T> mSortedItems;

    /**
     * X-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double mBeforeMeanFx;

    /**
     * Y-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double mBeforeMeanFy;

    /**
     * Z-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double mBeforeMeanFz;

    /**
     * X-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double mAfterMeanFx;

    /**
     * Y-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double mAfterMeanFy;

    /**
     * Z-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double mAfterMeanFz;

    /**
     * Constructor.
     */
    public BodyKinematicsSequence() {
    }

    /**
     * Constructor.
     *
     * @param items list of items containing body kinematics to be kept into
     *              this sequence.
     */
    public BodyKinematicsSequence(final List<T> items) {
        mItems = items;
    }

    /**
     * Constructor.
     *
     * @param beforeMeanFx x-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFy y-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFz z-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFx  x-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFy  y-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFz  z-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     */
    public BodyKinematicsSequence(
            final double beforeMeanFx,
            final double beforeMeanFy,
            final double beforeMeanFz,
            final double afterMeanFx,
            final double afterMeanFy,
            final double afterMeanFz) {
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanFx, beforeMeanFy, beforeMeanFz);
        setAfterMeanSpecificForceCoordinates(
                afterMeanFx, afterMeanFy, afterMeanFz);
    }

    /**
     * Constructor.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param afterMeanSpecificForceX x-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     * @param afterMeanSpecificForceY y-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     */
    public BodyKinematicsSequence(
            final Acceleration beforeMeanSpecificForceX,
            final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ,
            final Acceleration afterMeanSpecificForceX,
            final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX,
                beforeMeanSpecificForceY,
                beforeMeanSpecificForceZ);
        setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX,
                afterMeanSpecificForceY,
                afterMeanSpecificForceZ);
    }

    /**
     * @param items  list of items containing body kinematics to be kept into
     *               this sequence.
     * @param beforeMeanFx x-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFy y-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFz z-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFx  x-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFy  y-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFz  z-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     */
    public BodyKinematicsSequence(
            final List<T> items,
            final double beforeMeanFx,
            final double beforeMeanFy,
            final double beforeMeanFz,
            final double afterMeanFx,
            final double afterMeanFy,
            final double afterMeanFz) {
        this(items);
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanFx, beforeMeanFy, beforeMeanFz);
        setAfterMeanSpecificForceCoordinates(
                afterMeanFx, afterMeanFy, afterMeanFz);
    }

    /**
     * Constructor.
     *
     * @param items              list of items containing body kinematics to be kept into
     *                           this sequence.
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force during
     *                           the static period happening right before this
     *                           sequence was measured.
     * @param afterMeanSpecificForceX x-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     * @param afterMeanSpecificForceY y-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force during
     *                           the static period happening right after this
     *                           sequence was measured.
     */
    public BodyKinematicsSequence(
            final List<T> items,
            final Acceleration beforeMeanSpecificForceX,
            final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ,
            final Acceleration afterMeanSpecificForceX,
            final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        this(items);
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX,
                beforeMeanSpecificForceY,
                beforeMeanSpecificForceZ);
        setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX,
                afterMeanSpecificForceY,
                afterMeanSpecificForceZ);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyKinematicsSequence(
            final BodyKinematicsSequence<T> input) {
        copyFrom(input);
    }

    /**
     * Gets items in this sequence ordered by ascending timestamp.
     *
     * @param result instance where sorted items will be stored.
     * @return true if sorted items could be retrieved, false if no items
     * are available.
     */
    public boolean getSortedItems(final List<T> result) {
        // already sorted items are available.
        if (mSortedItems != null) {
            result.clear();
            result.addAll(mSortedItems);
            return true;
        }

        if (mItems != null) {
            mSortedItems = new ArrayList<>(mItems);
            Collections.sort(mSortedItems, new Comparator<T>() {
                @Override
                public int compare(final T o1, final T o2) {
                    final double t1 = o1.getTimestampSeconds();
                    final double t2 = o2.getTimestampSeconds();
                    return Double.compare(t1, t2);
                }
            });

            result.clear();
            result.addAll(mSortedItems);
            return true;
        }

        return false;
    }

    /**
     * Gets items in this sequence ordered by ascending timestamp.
     *
     * @return a new list containing sorted items or null if sorted items
     * could not be retrieved..
     */
    public List<T> getSortedItems() {
        final List<T> result = new ArrayList<>();
        if (getSortedItems(result)) {
            return result;
        } else {
            return null;
        }
    }

    /**
     * Sets list of items containing body kinematics to be kept into this
     * sequence.
     *
     * @param items items to be kept.
     */
    public void setItems(final List<T> items) {
        mItems = items;
        mSortedItems = null;
    }

    /**
     * Gets number of items in this sequence.
     *
     * @return number of items in this sequence.
     */
    public int getItemsCount() {
        return mItems != null ? mItems.size() : 0;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return x-coordinate of mean specific force.
     */
    public double getBeforeMeanFx() {
        return mBeforeMeanFx;
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFx x-coordinate of mean specific force.
     */
    public void setBeforeMeanFx(final double beforeMeanFx) {
        mBeforeMeanFx = beforeMeanFx;
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return y-coordinate of mean specific force.
     */
    public double getBeforeMeanFy() {
        return mBeforeMeanFy;
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFy y-coordinate of mean specific force.
     */
    public void setBeforeMeanFy(final double beforeMeanFy) {
        mBeforeMeanFy = beforeMeanFy;
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return z-coordinate of mean specific force.
     */
    public double getBeforeMeanFz() {
        return mBeforeMeanFz;
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFz z-coordinate of mean specific force.
     */
    public void setBeforeMeanFz(final double beforeMeanFz) {
        mBeforeMeanFz = beforeMeanFz;
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFx x-coordinate of mean specific force.
     * @param beforeMeanFy y-coordinate of mean specific force.
     * @param beforeMeanFz z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceCoordinates(
            final double beforeMeanFx,
            final double beforeMeanFy,
            final double beforeMeanFz) {
        mBeforeMeanFx = beforeMeanFx;
        mBeforeMeanFy = beforeMeanFy;
        mBeforeMeanFz = beforeMeanFz;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result x-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceX(final Acceleration result) {
        result.setValue(mBeforeMeanFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return x-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceX() {
        return new Acceleration(mBeforeMeanFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceX(
            final Acceleration beforeMeanSpecificForceX) {
        mBeforeMeanFx = convertAcceleration(beforeMeanSpecificForceX);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result y-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceY(final Acceleration result) {
        result.setValue(mBeforeMeanFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return y-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceY() {
        return new Acceleration(mBeforeMeanFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceY(
            final Acceleration beforeMeanSpecificForceY) {
        mBeforeMeanFy = convertAcceleration(beforeMeanSpecificForceY);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result z-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceZ(final Acceleration result) {
        result.setValue(mBeforeMeanFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return z-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceZ() {
        return new Acceleration(mBeforeMeanFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceZ(
            final Acceleration beforeMeanSpecificForceZ) {
        mBeforeMeanFz = convertAcceleration(beforeMeanSpecificForceZ);
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceCoordinates(
            final Acceleration beforeMeanSpecificForceX,
            final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ) {
        setBeforeMeanSpecificForceX(beforeMeanSpecificForceX);
        setBeforeMeanSpecificForceY(beforeMeanSpecificForceY);
        setBeforeMeanSpecificForceZ(beforeMeanSpecificForceZ);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return x-coordinate of mean specific force.
     */
    public double getAfterMeanFx() {
        return mAfterMeanFx;
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFx x-coordinate of mean specific force.
     */
    public void setAfterMeanFx(final double afterMeanFx) {
        mAfterMeanFx = afterMeanFx;
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return y-coordinate of mean specific force.
     */
    public double getAfterMeanFy() {
        return mAfterMeanFy;
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFy y-coordinate of mean specific force.
     */
    public void setAfterMeanFy(final double afterMeanFy) {
        mAfterMeanFy = afterMeanFy;
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return z-coordinate of mean specific force.
     */
    public double getAfterMeanFz() {
        return mAfterMeanFz;
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFz z-coordinate of mean specific force.
     */
    public void setAfterMeanFz(final double afterMeanFz) {
        mAfterMeanFz = afterMeanFz;
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFx x-coordinate of mean specific force.
     * @param afterMeanFy y-coordinate of mean specific force.
     * @param afterMeanFz z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceCoordinates(
            final double afterMeanFx,
            final double afterMeanFy,
            final double afterMeanFz) {
        mAfterMeanFx = afterMeanFx;
        mAfterMeanFy = afterMeanFy;
        mAfterMeanFz = afterMeanFz;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result x-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceX(final Acceleration result) {
        result.setValue(mAfterMeanFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return x-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceX() {
        return new Acceleration(mAfterMeanFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceX x-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceX(
            final Acceleration afterMeanSpecificForceX) {
        mAfterMeanFx = convertAcceleration(afterMeanSpecificForceX);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result y-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceY(final Acceleration result) {
        result.setValue(mAfterMeanFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return y-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceY() {
        return new Acceleration(mAfterMeanFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceY y-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceY(
            final Acceleration afterMeanSpecificForceY) {
        mAfterMeanFy = convertAcceleration(afterMeanSpecificForceY);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result z-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceZ(final Acceleration result) {
        result.setValue(mAfterMeanFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return z-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceZ() {
        return new Acceleration(mAfterMeanFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceZ(
            final Acceleration afterMeanSpecificForceZ) {
        mAfterMeanFz = convertAcceleration(afterMeanSpecificForceZ);
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceX x-coordinate of mean specific force.
     * @param afterMeanSpecificForceY y-coordinate of mean specific force.
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceCoordinates(
            final Acceleration afterMeanSpecificForceX,
            final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        setAfterMeanSpecificForceX(afterMeanSpecificForceX);
        setAfterMeanSpecificForceY(afterMeanSpecificForceY);
        setAfterMeanSpecificForceZ(afterMeanSpecificForceZ);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(BodyKinematicsSequence<T> input) {
        if (input.mItems != null) {
            mItems = cloneList(input.mItems);
        } else {
            mItems = null;
        }
        if (input.mSortedItems != null) {
            mSortedItems = cloneList(input.mSortedItems);
        } else {
            mSortedItems = null;
        }

        mBeforeMeanFx = input.mBeforeMeanFx;
        mBeforeMeanFy = input.mBeforeMeanFy;
        mBeforeMeanFz = input.mBeforeMeanFz;

        mAfterMeanFx = input.mAfterMeanFx;
        mAfterMeanFy = input.mAfterMeanFy;
        mAfterMeanFz = input.mAfterMeanFz;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyKinematicsSequence<T> output) {
        output.copyFrom(this);
    }

    /***
     * Checks if provided instance is a BodyKinematicsSequence2 having exactly
     * the same contents as this instance.
     *
     * @param o object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        BodyKinematicsSequence<?> that = (BodyKinematicsSequence<?>) o;
        return Double.compare(that.mBeforeMeanFx, mBeforeMeanFx) == 0 &&
                Double.compare(that.mBeforeMeanFy, mBeforeMeanFy) == 0 &&
                Double.compare(that.mBeforeMeanFz, mBeforeMeanFz) == 0 &&
                Double.compare(that.mAfterMeanFx, mAfterMeanFx) == 0 &&
                Double.compare(that.mAfterMeanFy, mAfterMeanFy) == 0 &&
                Double.compare(that.mAfterMeanFz, mAfterMeanFz) == 0 &&
                Objects.equals(mItems, that.mItems) &&
                Objects.equals(mSortedItems, that.mSortedItems);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mItems, mSortedItems,
                mBeforeMeanFx, mBeforeMeanFy, mBeforeMeanFz,
                mAfterMeanFx, mAfterMeanFy, mAfterMeanFz);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        //noinspection unchecked
        final BodyKinematicsSequence<T> result =
                (BodyKinematicsSequence<T>) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Clones a list of {@link TimedBodyKinematics}.
     *
     * @param list list to be cloned.
     * @return cloned list.
     */
    @SuppressWarnings("unchecked")
    private List<T> cloneList(final List<T> list) {
        // constructor with list only creates a new list containing the
        // same instances as the original list.
        // ArrayList is publicly Cloneable, so we clone it to get copies
        // of the elements contained within.

        final ArrayList<T> result = new ArrayList<>();
        for (T item : list) {
            if (item instanceof StandardDeviationTimedBodyKinematics) {
                final StandardDeviationTimedBodyKinematics newItem =
                        new StandardDeviationTimedBodyKinematics();
                newItem.copyFrom(item);
                result.add((T) newItem);
            } else {
                final TimedBodyKinematics newItem =
                        new TimedBodyKinematics(item);
                result.add((T) newItem);
            }
        }

        return result;
    }

    /**
     * Converts an acceleration instance into its corresponding value
     * expressed in meters per squared second.
     *
     * @param acceleration an acceleration to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }
}
