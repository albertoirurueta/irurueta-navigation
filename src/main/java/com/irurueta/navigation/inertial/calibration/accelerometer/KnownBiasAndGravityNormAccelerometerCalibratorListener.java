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
package com.irurueta.navigation.inertial.calibration.accelerometer;

/**
 * Contains listener for accelerometer calibrator when the same gravity norm is known
 * for all measurements, bias is known and orientation is unknown.
 */
public interface KnownBiasAndGravityNormAccelerometerCalibratorListener extends
        BaseBiasGravityNormAccelerometerCalibratorListener<KnownBiasAndGravityNormAccelerometerCalibrator> {
}
