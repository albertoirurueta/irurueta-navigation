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

/**
 * Contains approximate bias (or hard iron for the magnetometer case) estimators.
 *
 * Bias can be approximately estimated for accelerometers, gyroscopes and
 * magnetometers when body remains static and its position respect to Earth
 * and orientation is approximately known.
 *
 * Estimators in this package assume that any cross coupling error can be
 * neglected, hence estimated biases are approximate, however these
 * approximate values can be used by some non-linear calibrators to obtain
 * more accurate results such as:
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownPositionAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownFrameAccelerometerCalibrator} and
 * any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownGravityNormAccelerometerCalibrator}
 * and any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownPositionAccelerometerCalibrator} and
 * any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownFrameGyroscopeNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustEasyGyroscopeCalibrator} and any of its
 * subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownFrameGyroscopeCalibrator} and any of its
 * subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustTurntableGyroscopeCalibrator} and any of its
 * subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownFrameMagnetometerNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownFrameMagnetometerCalibrator} and any
 * of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownPositionAndInstantMagnetometerCalibrator}
 * and any of its subclasses.
 *
 *
 * On the other hand, it can be simply be assumed that estimated biases are
 * accurate enough and can be used by calibrators such as:
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndGravityNormAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndPositionAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownBiasAndFrameAccelerometerCalibrator}
 * and any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownBiasAndGravityNormAccelerometerCalibrator}
 * and any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownBiasAndPositionAccelerometerCalibrator}
 * and any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasEasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasTurntableGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasAndFrameGyroscopeCalibrator} and any
 * of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasEasyGyroscopeCalibrator} and any of
 * its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasTurntableGyroscopeCalibrator} and any
 * of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownHardIronAndFrameMagnetometerCalibrator}
 * and any of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownHardIronPositionAndInstantMagnetometerCalibrator}
 * and any of its subclasses.
 */
package com.irurueta.navigation.inertial.calibration.bias;