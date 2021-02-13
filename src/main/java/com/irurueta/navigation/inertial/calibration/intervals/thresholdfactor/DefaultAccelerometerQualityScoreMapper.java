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
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;

/**
 * Default implementation to map a given measurement into a given quality score to be
 * used for accelerometer calibration.
 */
public class DefaultAccelerometerQualityScoreMapper implements
        QualityScoreMapper<StandardDeviationBodyKinematics> {

    /**
     * Maps a given value corresponding to a measurement, into a given quality score.
     *
     * @param value type of measurement.
     * @return mapped quality score.
     */
    @Override
    public double map(final StandardDeviationBodyKinematics value) {
        return 1.0 / (1.0 + value.getSpecificForceStandardDeviation() + value.getAngularRateStandardDeviation());
    }
}
