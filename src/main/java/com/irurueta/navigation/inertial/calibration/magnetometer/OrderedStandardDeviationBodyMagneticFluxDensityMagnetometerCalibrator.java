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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;

import java.util.List;

/**
 * Defines a magnetometer calibrator using ordered lists of
 * {@link StandardDeviationBodyMagneticFluxDensity} measurements.
 */
public interface OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator extends
        MagnetometerCalibrator {

    /**
     * Gets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @return collection of body magnetic flux density measurements at
     * a known position and timestamp with unknown orientations.
     */
    List<StandardDeviationBodyMagneticFluxDensity> getMeasurements();

    /**
     * Sets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements at a known position and timestamp
     *                     with unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    void setMeasurements(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements)
            throws LockedException;
}
