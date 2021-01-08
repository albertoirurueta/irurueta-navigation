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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;

/**
 * Defines a source for position uncertainty.
 */
public interface PositionUncertaintySource {

    /**
     * Gets standard deviation of position noise or uncertainty expressed in
     * meters (m).
     * Position noise can be measured using a {@link RandomWalkEstimator}, which
     * is appropriate to create a {@link INSLooselyCoupledKalmanConfig}.
     * However, to create a {@link INSLooselyCoupledKalmanInitializerConfig},
     * typically position, velocity and attitude are externally measured by other
     * means introducing a different amount of uncertainty.
     *
     * @return standard deviation of position expressed in meters (m).
     */
    double getPositionStandardDeviation();
}
