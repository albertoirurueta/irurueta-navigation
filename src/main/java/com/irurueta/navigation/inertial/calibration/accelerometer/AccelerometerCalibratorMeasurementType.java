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

import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;

/**
 * Contains measurement types supported by an accelerometer calibrator.
 */
public enum AccelerometerCalibratorMeasurementType {
    /**
     * When a calibrator uses {@link FrameBodyKinematics} measurements.
     */
    FRAME_BODY_KINEMATICS,

    /**
     * When a calibrator uses {@link StandardDeviationBodyKinematics} measurements.
     */
    STANDARD_DEVIATION_BODY_KINEMATICS,

    /**
     * When a calibrator uses {@link StandardDeviationFrameBodyKinematics} measurements.
     */
    STANDARD_DEVIATION_FRAME_BODY_KINEMATICS
}
