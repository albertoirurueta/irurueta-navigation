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

import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;

import java.util.List;

/**
 * Default implementation to map a given measurement into a given quality score to be
 * used for gyroscope calibration.
 */
public class DefaultGyroscopeQualityScoreMapper implements
        QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> {

    /**
     * Maps a given value corresponding to a generated measurement, into a given quality score.
     *
     * @param value type of measurement.
     * @return mapped quality score.
     */
    @Override
    public double map(final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> value) {
        final List<StandardDeviationTimedBodyKinematics> items = value.getSortedItems();
        if (items != null) {
            final int count = value.getItemsCount();

            double avg = 0.0;
            for (StandardDeviationTimedBodyKinematics item : items) {
                avg += (item.getSpecificForceStandardDeviation() + item.getAngularRateStandardDeviation()) / count;
            }

            return 1.0 / (1.0 + avg);
        } else {
            return 0.0;
        }
    }
}
