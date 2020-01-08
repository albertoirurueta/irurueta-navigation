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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

/**
 * Generates body kinematic instances from true body kinematic values taking into
 * account provided IMU errors for a calibrated IMU.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/IMU_model.m
 */
public class BodyKinematicsGenerator {

    /**
     * Prevents instantiation of utility class.
     */
    private BodyKinematicsGenerator() { }

    /**
     * Generates uncalibrated body kinematics instances containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     * This method ignores IMU quantization levels.
     *
     * @param timeInterval   time interval between epochs.
     * @param trueKinematics collection of ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @return a new collection containing generated uncalibrated kinematics
     * for each provided ground-truth one.
     */
    public static Collection<BodyKinematics> generate(
            final Time timeInterval, final Collection<BodyKinematics> trueKinematics,
            final IMUErrors errors, final Random random) {
        return generate(convertTime(timeInterval), trueKinematics, errors, random);
    }

    /**
     * Generates uncalibrated body kinematics instances containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     * This method ignores IMU quantization levels.
     *
     * @param timeInterval   time interval between epochs.
     * @param trueKinematics collection of ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @param result         collection where generated uncalibrated kinematics
     *                       for each provided ground-truth one will be stored.
     */
    public static void generate(final Time timeInterval,
                                final Collection<BodyKinematics> trueKinematics,
                                final IMUErrors errors, final Random random,
                                final Collection<BodyKinematics> result) {
        generate(convertTime(timeInterval), trueKinematics, errors, random, result);
    }

    /**
     * Generates uncalibrated body kinematics instances containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     * This method ignores IMU quantization levels.
     *
     * @param timeInterval   time interval between epochs expressed in seconds (s).
     * @param trueKinematics collection of ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @return a new collection containing generated uncalibrated kinematics
     * for each provided ground-truth one.
     */
    public static Collection<BodyKinematics> generate(
            final double timeInterval, final Collection<BodyKinematics> trueKinematics,
            final IMUErrors errors, final Random random) {
        final List<BodyKinematics> result = new ArrayList<>();
        generate(timeInterval, trueKinematics, errors, random, result);
        return result;
    }

    /**
     * Generates uncalibrated body kinematics instances containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     * This method ignores IMU quantization levels.
     *
     * @param timeInterval   time interval between epochs expressed in seconds (s).
     * @param trueKinematics collection of ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @param result         collection where generated uncalibrated kinematics
     *                       for each provided ground-truth one will be stored.
     */
    public static void generate(final double timeInterval,
                                final Collection<BodyKinematics> trueKinematics,
                                final IMUErrors errors, final Random random,
                                final Collection<BodyKinematics> result) {
        try {
            final Matrix trueFibb = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix ma = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();
            final Matrix ba = errors.getAccelerometerBiasesAsMatrix();
            final Matrix trueOmegaIbb = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix mg = errors.getGyroScaleFactorAndCrossCouplingErrors();
            final Matrix bg = errors.getGyroBiasesAsMatrix();
            final Matrix gg = errors.getGyroGDependentBiases();
            final Matrix identity = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            final Matrix tmp33 = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            final Matrix tmp31a = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix tmp31b = new Matrix(BodyKinematics.COMPONENTS, 1);

            for (final BodyKinematics k : trueKinematics) {
                final BodyKinematics r = new BodyKinematics();

                internalGenerate(timeInterval, k, errors, random,
                        null, r, null,
                        trueFibb, ma, ba, trueOmegaIbb, mg, bg, gg, identity,
                        tmp33, tmp31a, tmp31b);

                result.add(r);
            }

        } catch (WrongSizeException ignore) {
            // never happens
        }

    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval   time interval between epochs expressed in seconds (s).
     * @param trueKinematics ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @return uncalibrated body kinematics.
     */
    public static BodyKinematics generate(final Time timeInterval,
                                          final BodyKinematics trueKinematics,
                                          final IMUErrors errors, final Random random) {
        return generate(convertTime(timeInterval), trueKinematics, errors, random);
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval   time interval between epochs.
     * @param trueKinematics ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @param result         instance where uncalibrated body kinematics will be stored.
     */
    public static void generate(final Time timeInterval,
                                final BodyKinematics trueKinematics,
                                final IMUErrors errors, final Random random,
                                final BodyKinematics result) {
        generate(convertTime(timeInterval), trueKinematics,
                errors, random, result);
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval   time interval between epochs expressed in seconds (s).
     * @param trueKinematics ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @return uncalibrated body kinematics.
     */
    public static BodyKinematics generate(final double timeInterval,
                                          final BodyKinematics trueKinematics,
                                          final IMUErrors errors,
                                          final Random random) {
        final BodyKinematics result = new BodyKinematics();
        generate(timeInterval, trueKinematics, errors, random, result);
        return result;
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval   time interval between epochs expressed in seconds (s).
     * @param trueKinematics ground-truth kinematics.
     * @param errors         IMU errors containing calibration data.
     * @param random         a random number generator to generate noise.
     * @param result         instance where uncalibrated body kinematics will be stored.
     */
    public static void generate(final double timeInterval,
                                final BodyKinematics trueKinematics,
                                final IMUErrors errors, final Random random,
                                final BodyKinematics result) {
        generate(timeInterval, trueKinematics, errors, random,
                null, result, null);
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval             time interval between epochs.
     * @param trueKinematics           ground-truth kinematics.
     * @param errors                   IMU errors containing calibration data.
     * @param random                   a random number generator to generate noise.
     * @param oldQuantizationResiduals previous quantization residuals from previous
     *                                 executions. Optional. If provided, must have
     *                                 length 6, if not provided quantization levels
     *                                 are ignored.
     * @param quantizationResiduals    generated quantization residuals. Optional.
     *                                 If provided, must have length 6.
     * @return generated uncalibrated body kinematics.
     * @throws IllegalArgumentException if either oldQuantizationResiduals or
     *                                  quantizationResiduals are not length 6.
     */
    public static BodyKinematics generate(final Time timeInterval,
                                          final BodyKinematics trueKinematics,
                                          final IMUErrors errors, final Random random,
                                          final double[] oldQuantizationResiduals,
                                          final double[] quantizationResiduals) {
        return generate(convertTime(timeInterval), trueKinematics, errors, random,
                oldQuantizationResiduals, quantizationResiduals);
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval             time interval between epochs.
     * @param trueKinematics           ground-truth kinematics.
     * @param errors                   IMU errors containing calibration data.
     * @param random                   a random number generator to generate noise.
     * @param oldQuantizationResiduals previous quantization residuals from previous
     *                                 executions. Optional. If provided, must have
     *                                 length 6, if not provided quantization levels
     *                                 are ignored.
     * @param result                   instance where uncalibrated body kinematics will be stored.
     * @param quantizationResiduals    generated quantization residuals. Optional.
     *                                 If provided, must have length 6.
     * @throws IllegalArgumentException if either oldQuantizationResiduals or
     *                                  quantizationResiduals are not length 6.
     */
    public static void generate(final Time timeInterval,
                                final BodyKinematics trueKinematics,
                                final IMUErrors errors, final Random random,
                                final double[] oldQuantizationResiduals,
                                final BodyKinematics result,
                                final double[] quantizationResiduals) {
        generate(convertTime(timeInterval), trueKinematics, errors, random,
                oldQuantizationResiduals, result, quantizationResiduals);
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval             time interval between epochs expressed in seconds (s).
     * @param trueKinematics           ground-truth kinematics.
     * @param errors                   IMU errors containing calibration data.
     * @param random                   a random number generator to generate noise.
     * @param oldQuantizationResiduals previous quantization residuals from previous
     *                                 executions. Optional. If provided, must have
     *                                 length 6, if not provided quantization levels
     *                                 are ignored.
     * @param quantizationResiduals    generated quantization residuals. Optional.
     *                                 If provided, must have length 6.
     * @return generated uncalibrated body kinematics.
     * @throws IllegalArgumentException if either oldQuantizationResiduals or
     *                                  quantizationResiduals are not length 6.
     */
    public static BodyKinematics generate(final double timeInterval,
                                          final BodyKinematics trueKinematics,
                                          final IMUErrors errors, final Random random,
                                          final double[] oldQuantizationResiduals,
                                          final double[] quantizationResiduals) {
        final BodyKinematics result = new BodyKinematics();
        generate(timeInterval, trueKinematics, errors, random,
                oldQuantizationResiduals, result, quantizationResiduals);
        return result;
    }

    /**
     * Generates an uncalibrated body kinematics instance containing a certain level
     * of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval             time interval between epochs expressed in seconds (s).
     * @param trueKinematics           ground-truth kinematics.
     * @param errors                   IMU errors containing calibration data.
     * @param random                   a random number generator to generate noise.
     * @param oldQuantizationResiduals previous quantization residuals from previous
     *                                 executions. Optional. If provided, must have
     *                                 length 6, if not provided quantization levels
     *                                 are ignored.
     * @param result                   instance where uncalibrated body kinematics will be stored.
     * @param quantizationResiduals    generated quantization residuals. Optional.
     *                                 If provided, must have length 6.
     * @throws IllegalArgumentException if either oldQuantizationResiduals or
     *                                  quantizationResiduals are not length 6.
     */
    public static void generate(final double timeInterval,
                                final BodyKinematics trueKinematics,
                                final IMUErrors errors, final Random random,
                                final double[] oldQuantizationResiduals,
                                final BodyKinematics result,
                                final double[] quantizationResiduals) {
        try {
            final Matrix trueFibb = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix ma = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();
            final Matrix ba = errors.getAccelerometerBiasesAsMatrix();
            final Matrix trueOmegaIbb = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix mg = errors.getGyroScaleFactorAndCrossCouplingErrors();
            final Matrix bg = errors.getGyroBiasesAsMatrix();
            final Matrix gg = errors.getGyroGDependentBiases();
            final Matrix identity = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            final Matrix tmp33 = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            final Matrix tmp31a = new Matrix(BodyKinematics.COMPONENTS, 1);
            final Matrix tmp31b = new Matrix(BodyKinematics.COMPONENTS, 1);
            internalGenerate(timeInterval, trueKinematics, errors, random,
                    oldQuantizationResiduals, result, quantizationResiduals,
                    trueFibb, ma, ba, trueOmegaIbb, mg, bg, gg, identity, tmp33, tmp31a,
                    tmp31b);
        } catch (WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Internally generates an uncalibrated body kinematics instance containing a certain
     * level of noise for provided ground-truth body kinematics and IMU errors.
     *
     * @param timeInterval             time interval between epochs expressed in seconds (s).
     * @param trueKinematics           ground-truth kinematics.
     * @param errors                   IMU errors containing calibration data.
     * @param random                   a random number generator to generate noise.
     * @param oldQuantizationResiduals previous quantization residuals from previous
     *                                 executions. Optional. If provided, must have
     *                                 length 6, if not provided quantization levels
     *                                 are ignored.
     * @param result                   instance where uncalibrated body kinematics will be stored.
     * @param quantizationResiduals    generated quantization residuals. Optional.
     *                                 If provided, must have length 6.
     * @param trueFibb                 3x1 matrix to be reused for specific force storage.
     * @param ma                       3x3 matrix to be reused for accelerometer scaling and
     *                                 cross coupling errors.
     * @param ba                       3x1 matrix to be reused for accelerometer biases.
     * @param trueOmegaibb             3x1 matrix to be reused for angular rates storage.
     * @param mg                       3x3 matrix to be reused for gyro scaling and cross
     *                                 coupling errors.
     * @param bg                       3x1 matrix to be reused for gyro biases.
     * @param gg                       3x3 matrix to be reused for gyro dependan cross
     *                                 coupling errors.
     * @param identity                 3x3 identity matrix to be reused.
     * @param tmp33                    3x3 matrix to be reused.
     * @param tmp31a                   3x1 matrix to be reused.
     * @param tmp31b                   3x1 matrix to be reused.
     * @throws WrongSizeException       if any of provided matrices has invalid size.
     * @throws IllegalArgumentException if either oldQuantizationResiduals or
     *                                  quantizationResiduals are not length 6.
     */
    private static void internalGenerate(final double timeInterval,
                                         final BodyKinematics trueKinematics,
                                         final IMUErrors errors, final Random random,
                                         final double[] oldQuantizationResiduals,
                                         final BodyKinematics result,
                                         final double[] quantizationResiduals,
                                         final Matrix trueFibb,
                                         final Matrix ma,
                                         final Matrix ba,
                                         final Matrix trueOmegaibb,
                                         final Matrix mg,
                                         final Matrix bg,
                                         final Matrix gg,
                                         final Matrix identity,
                                         final Matrix tmp33,
                                         final Matrix tmp31a,
                                         final Matrix tmp31b) throws WrongSizeException {

        int comp2 = 2 * BodyKinematics.COMPONENTS;
        if (oldQuantizationResiduals != null &&
                oldQuantizationResiduals.length != comp2) {
            throw new IllegalArgumentException();
        }
        if (quantizationResiduals != null &&
                quantizationResiduals.length != comp2) {
            throw new IllegalArgumentException();
        }

        final double accelNoiseX;
        final double accelNoiseY;
        final double accelNoiseZ;
        final double gyroNoiseX;
        final double gyroNoiseY;
        final double gyroNoiseZ;
        if (timeInterval > 0.0) {
            final double sqrtTimeInterval = Math.sqrt(timeInterval);

            final double accelNoiseRootPSD = errors.getAccelerometerNoiseRootPSD();
            final double accelStd = accelNoiseRootPSD / sqrtTimeInterval;

            accelNoiseX = random.nextGaussian() * accelStd;
            accelNoiseY = random.nextGaussian() * accelStd;
            accelNoiseZ = random.nextGaussian() * accelStd;

            final double gyroNoiseRootPSD = errors.getGyroNoiseRootPSD();
            final double gyroStd = gyroNoiseRootPSD / sqrtTimeInterval;

            gyroNoiseX = random.nextGaussian() * gyroStd;
            gyroNoiseY = random.nextGaussian() * gyroStd;
            gyroNoiseZ = random.nextGaussian() * gyroStd;
        } else {
            accelNoiseX = 0.0;
            accelNoiseY = 0.0;
            accelNoiseZ = 0.0;

            gyroNoiseX = 0.0;
            gyroNoiseY = 0.0;
            gyroNoiseZ = 0.0;
        }

        // Calculate accelerometer and gyro outputs using (4.16) and (4.17)
        trueKinematics.asSpecificForceMatrix(trueFibb);
        trueKinematics.asAngularRateMatrix(trueOmegaibb);

        identity.add(ma, tmp33);
        tmp33.multiply(trueFibb, tmp31a);
        tmp31a.add(ba);

        final double uqFibbX = tmp31a.getElementAtIndex(0) + accelNoiseX;
        final double uqFibbY = tmp31a.getElementAtIndex(1) + accelNoiseY;
        final double uqFibbZ = tmp31a.getElementAtIndex(2) + accelNoiseZ;

        identity.add(mg, tmp33);
        tmp33.multiply(trueOmegaibb, tmp31a);
        tmp31a.add(bg);

        gg.multiply(trueFibb, tmp31b);
        tmp31a.add(tmp31b);

        final double uqOmegaIbbX = tmp31a.getElementAtIndex(0) + gyroNoiseX;
        final double uqOmegaIbbY = tmp31a.getElementAtIndex(1) + gyroNoiseY;
        final double uqOmegaIbbZ = tmp31a.getElementAtIndex(2) + gyroNoiseZ;

        // Quantize accelerometer outputs
        if (errors.getAccelerometerQuantizationLevel() > 0.0
                && oldQuantizationResiduals != null) {
            final double accelQuantLevel = errors.getAccelerometerQuantizationLevel();
            final double fx = accelQuantLevel
                    * Math.round((uqFibbX + oldQuantizationResiduals[0])
                    / accelQuantLevel);
            final double fy = accelQuantLevel
                    * Math.round((uqFibbY + oldQuantizationResiduals[1])
                    / accelQuantLevel);
            final double fz = accelQuantLevel
                    * Math.round((uqFibbZ + oldQuantizationResiduals[2])
                    / accelQuantLevel);

            result.setSpecificForceCoordinates(fx, fy, fz);

            if (quantizationResiduals != null) {
                quantizationResiduals[0] = uqFibbX + oldQuantizationResiduals[0] - fx;
                quantizationResiduals[1] = uqFibbY + oldQuantizationResiduals[1] - fy;
                quantizationResiduals[2] = uqFibbZ + oldQuantizationResiduals[2] - fz;
            }
        } else {
            result.setSpecificForceCoordinates(uqFibbX, uqFibbY, uqFibbZ);

            if (quantizationResiduals != null) {
                quantizationResiduals[0] = 0.0;
                quantizationResiduals[1] = 0.0;
                quantizationResiduals[2] = 0.0;
            }
        }

        // Quantize gyro outputs
        if (errors.getGyroQuantizationLevel() > 0.0
                && oldQuantizationResiduals != null) {
            final double gyroQuantLevel = errors.getGyroQuantizationLevel();
            final double omegaX = gyroQuantLevel
                    * Math.round((uqOmegaIbbX + oldQuantizationResiduals[3])
                    / gyroQuantLevel);
            final double omegaY = gyroQuantLevel
                    * Math.round((uqOmegaIbbY + oldQuantizationResiduals[4])
                    / gyroQuantLevel);
            final double omegaZ = gyroQuantLevel
                    * Math.round((uqOmegaIbbZ + oldQuantizationResiduals[5])
                    / gyroQuantLevel);

            result.setAngularRateCoordinates(omegaX, omegaY, omegaZ);

            if (quantizationResiduals != null) {
                quantizationResiduals[3] = uqOmegaIbbX + oldQuantizationResiduals[3]
                        - omegaX;
                quantizationResiduals[4] = uqOmegaIbbY + oldQuantizationResiduals[4]
                        - omegaY;
                quantizationResiduals[5] = uqOmegaIbbZ + oldQuantizationResiduals[5]
                        - omegaZ;
            }
        } else {
            result.setAngularRateCoordinates(uqOmegaIbbX, uqOmegaIbbY, uqOmegaIbbZ);

            if (quantizationResiduals != null) {
                quantizationResiduals[3] = 0.0;
                quantizationResiduals[4] = 0.0;
                quantizationResiduals[5] = 0.0;
            }
        }
    }

    /**
     * Converts provided time instance to seconds.
     *
     * @param time time instance to be converted.
     * @return converted value.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }
}
