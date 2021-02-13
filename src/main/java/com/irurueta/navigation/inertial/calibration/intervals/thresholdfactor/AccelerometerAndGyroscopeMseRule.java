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

/**
 * Provides a rule or function to convert estimated accelerometer calibration
 * Mean Square Error (MSE) and estimated gyroscope calibration Mean Square Error
 * (MSE) into a single MSE value.
 * Typical implementation simply sums both MSE values, however, custom
 * implementations can be made to make one error more prevalent than the other, etc.
 */
public interface AccelerometerAndGyroscopeMseRule {

    /**
     * Evaluates provided accelerometer calibration MSE and gyroscope calibration
     * MSE in order to obtain a single MSE value representing both.
     *
     * @param accelerometerMse accelerometer calibration MSE.
     * @param gyroscopeMse     gyroscope calibration MSE.
     * @return single MSE value.
     */
    double evaluate(final double accelerometerMse, final double gyroscopeMse);
}
