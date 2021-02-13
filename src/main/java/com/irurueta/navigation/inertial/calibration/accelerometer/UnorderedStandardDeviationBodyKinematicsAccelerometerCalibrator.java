/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;

import java.util.Collection;

/**
 * Defines an accelerometer calibrator using unordered collections of
 * {@link StandardDeviationBodyKinematics} measurements.
 */
public interface UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator extends
        AccelerometerCalibrator {

    /**
     * Gets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @return collection of body kinematics measurements at a known position
     * with unknown orientations.
     */
    Collection<StandardDeviationBodyKinematics> getMeasurements();

    /**
     * Sets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @param measurements collection of body kinematics measurements at a
     *                     known position witn unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    void setMeasurements(
            final Collection<StandardDeviationBodyKinematics> measurements)
            throws LockedException;
}
