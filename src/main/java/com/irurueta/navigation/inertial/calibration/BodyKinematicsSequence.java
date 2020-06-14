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

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Contains a collection of items containing body kinematics
 * measurements ordered by the timestamp when the measurment was made.
 *
 * @param <T> a type of {@link TimedBodyKinematics}.
 */
public class BodyKinematicsSequence<T extends TimedBodyKinematics> {

    /**
     * List of items.
     * If items are provided unsorted, they are reordered by timestamp
     * on getter method.
     */
    private List<T> mItems;

    /**
     * Body attitude at the start of the sequence.
     * Must be a valid body to ECEF coordinate transformation.
     * This can be estimated using
     * {@link com.irurueta.navigation.inertial.estimators.AttitudeEstimator}
     * for a known Earth position, time instant, accelerometer measure
     * and magnetometer measure. Notice that both the accelerometer
     * and magnetometer should be previously calibrated before estimating
     * body attitude.
     * Alternatively, if the gyroscope is accurate enough (aviation grade),
     * a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator}
     * or a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator2}
     * can be used.
     * However, since this class is used mainly for gyroscope calibration,
     * leveling estimator should be discarded for initial body attitude
     * estimation.
     */
    private CoordinateTransformation mStartBodyAttitude;

    /**
     * Contains sorted list of items.
     * This list is kept for performance reasons to reduce the amount of
     * required sorting.
     */
    private List<T> mSortedItems;

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
     * @param startBodyAttitude body attitude at the start of the sequence.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided transformation is
     *                                                       not a body to ECEF transformation.
     */
    public BodyKinematicsSequence(
            final CoordinateTransformation startBodyAttitude)
            throws InvalidSourceAndDestinationFrameTypeException {
        setStartBodyAttitude(startBodyAttitude);
    }

    /**
     * Constructor.
     *
     * @param items             list of items containing body kinematics to be kept into
     *                          this sequence.
     * @param startBodyAttitude body attitude at the start of the sequence.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided transformation is
     *                                                       not a body to ECEF transformation.
     */
    public BodyKinematicsSequence(
            final List<T> items,
            final CoordinateTransformation startBodyAttitude)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(items);
        setStartBodyAttitude(startBodyAttitude);
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
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to ECEF transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidBodyToEcefCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return ECEFFrame.isValidCoordinateTransformation(c);
    }

    /**
     * Gets body attitude at the start of the sequence.
     * This can be estimated using
     * {@link com.irurueta.navigation.inertial.estimators.AttitudeEstimator}
     * for a known Earth position, time instant, accelerometer measure
     * and magnetometer measure. Notice that both the accelerometer
     * and magnetometer should be previously calibrated before estimating
     * body attitude.
     * Alternatively, if the gyroscope is accurate enough (aviation grade),
     * a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator}
     * or a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator2}
     * can be used.
     * However, since this class is used mainly for gyroscope calibration,
     * leveling estimator should be discarded for initial body attitude
     * estimation.
     *
     * @return body attitude at the start of the sequence.
     */
    public CoordinateTransformation getStartBodyAttitude() {
        return mStartBodyAttitude;
    }

    /**
     * Sets body attitude at the start of the sequence.
     * This can be estimated using
     * {@link com.irurueta.navigation.inertial.estimators.AttitudeEstimator}
     * for a known Earth position, time instant, accelerometer measure
     * and magnetometer measure. Notice that both the accelerometer
     * and magnetometer should be previously calibrated before estimating
     * body attitude.
     * Alternatively, if the gyroscope is accurate enough (aviation grade),
     * a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator}
     * or a {@link com.irurueta.navigation.inertial.estimators.LevelingEstimator2}
     * can be used.
     * However, since this class is used mainly for gyroscope calibration,
     * leveling estimator should be discarded for initial body attitude
     * estimation.
     *
     * @param startBodyAttitude body attitude at the start of the sequence.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided transformation is
     *                                                       not a body to ECEF transformation.
     */
    public void setStartBodyAttitude(
            final CoordinateTransformation startBodyAttitude)
            throws InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToEcefCoordinateTransformationMatrix(startBodyAttitude)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }
        mStartBodyAttitude = startBodyAttitude;
    }
}
