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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;

import java.util.List;

/**
 * Class in charge of performing integration steps of rotations.
 * This implementation uses a Runge-Kutta integration algorithm to obtain
 * accurate results on {@link EasyGyroscopeCalibrator}
 */
public class QuaternionIntegrator {

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at an initial attitude to obtain a final attitude.
     *
     * @param sequence        sequence of gyroscope measurements to be integrated.
     * @param initialAttitude (optional) initial attitude to be used. If null, then the
     *                        identity attitude will be used.
     * @param result          resulting rotation after integration.
     */
    public static void integrateGyroSequence(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final Quaternion initialAttitude,
            final Quaternion result) {

        if (initialAttitude != null) {
            // if provided initial attitude
            result.fromRotation(initialAttitude);
        } else {
            // if no initial attitude is provided, we use the identity
            result.setA(1.0);
            result.setB(0.0);
            result.setC(0.0);
            result.setD(0.0);
        }


        final List<StandardDeviationTimedBodyKinematics> sortedMeasurements =
                sequence.getSortedItems();

        boolean first = true;
        double previousTimestamp = 0.0;
        final double[] previousGyroData = new double[BodyKinematics.COMPONENTS];
        final double[] gyroData = new double[BodyKinematics.COMPONENTS];
        for (final StandardDeviationTimedBodyKinematics measurement : sortedMeasurements) {
            final BodyKinematics kinematics = measurement.getKinematics();

            gyroData[0] = kinematics.getAngularRateX();
            gyroData[1] = kinematics.getAngularRateY();
            gyroData[2] = kinematics.getAngularRateZ();

            if (first) {
                previousTimestamp = measurement.getTimestampSeconds();

                // copy gyroData to previousGyroData
                System.arraycopy(gyroData, 0, previousGyroData, 0, gyroData.length);

                first = false;
                continue;
            }

            final double timestamp = measurement.getTimestampSeconds();


            final double dt = timestamp - previousTimestamp;

            quatIntegrationStepRK4(result, previousGyroData, gyroData, dt, result);


            // prepare data for next iteration

            // copy gyroData to previousGyroData
            System.arraycopy(gyroData, 0, previousGyroData, 0, gyroData.length);
            // copy timestamp to previous timestamp
            previousTimestamp = timestamp;
        }
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at the identity attitude to obtain a final attitude.
     *
     * @param sequence sequence of gyroscope measurements to be integrated.
     * @param result   resulting rotation after integration.
     */
    public static void integrateGyroSequence(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final Quaternion result) {
        integrateGyroSequence(sequence, null, result);
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at an initial attitude to obtain a final attitude.
     *
     * @param sequence        sequence of gyroscope measurements to be integrated.
     * @param initialAttitude (optional) initial attitude to be used. If null, then the
     *                        identity attitude will be used.
     * @return resulting rotation after integration.
     */
    public static Quaternion integrateGyroSequenceAndReturnNew(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final Quaternion initialAttitude) {
        final Quaternion result = new Quaternion();
        integrateGyroSequence(sequence, initialAttitude, result);
        return result;
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at the identity attitude to obtain a final attitude.
     *
     * @param sequence sequence of gyroscope measurements to be integrated.
     * @return resulting rotation after integration.
     */
    public static Quaternion integrateGyroSequenceAndReturnNew(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
        final Quaternion result = new Quaternion();
        integrateGyroSequence(sequence, result);
        return result;
    }

    /**
     * Normalize an input matrix corresponding to a quaterniont into a unit
     * vector.
     *
     * @param quaternion matrix containing quaternion values to be normalized.
     */
    private static void normalizeQuaternion(final Matrix quaternion) {
        final double norm = Utils.normF(quaternion);
        quaternion.multiplyByScalar(1.0 / norm);
    }

    /**
     * Gets skew symmetric matrix representation of angular speed.
     *
     * @param omega  matrix containing angular speed components. Must be 3x1.
     * @param result computed skew symmetricmatrix representation. Must be 4x4.
     */
    private static void computeOmegaSkew(final Matrix omega, final Matrix result) {
        final double omega0 = omega.getElementAtIndex(0);
        final double omega1 = omega.getElementAtIndex(1);
        final double omega2 = omega.getElementAtIndex(2);

        result.setElementAtIndex(0, 0.0);
        result.setElementAtIndex(1, omega0);
        result.setElementAtIndex(2, omega1);
        result.setElementAtIndex(3, omega2);

        result.setElementAtIndex(4, -omega0);
        result.setElementAtIndex(5, 0.0);
        result.setElementAtIndex(6, -omega2);
        result.setElementAtIndex(7, omega1);

        result.setElementAtIndex(8, -omega1);
        result.setElementAtIndex(9, omega2);
        result.setElementAtIndex(10, 0.0);
        result.setElementAtIndex(11, -omega0);

        result.setElementAtIndex(12, -omega2);
        result.setElementAtIndex(13, -omega1);
        result.setElementAtIndex(14, omega0);
        result.setElementAtIndex(15, 0.0);
    }

    /**
     * Performs a RK4 Runge-Kutta integration step.
     *
     * @param quat   the input 4D vector representing the initial rotation as
     *               quaternion values. Must be 4x1.
     * @param omega0 initial rotational velocity at time t0 expressed in radians
     *               per second (rad/s). Must be 3x1.
     * @param omega1 final rotational velocity at time t1 expressed in radians
     *               per second (rad/s). Must be 3x1.
     * @param dt     time step expressed in seconds (t1 - t0).
     * @param result resulting final rotation. Must be 4x1.
     */
    private static void quatIntegrationStepRK4(
            final Matrix quat,
            final Matrix omega0,
            final Matrix omega1,
            final double dt,
            final Matrix result) {
        try {
            final Matrix omega01 = omega0.addAndReturnNew(omega1);
            omega01.multiplyByScalar(0.5);

            final Matrix k1 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k2 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k3 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k4 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix tmpQ = new Matrix(Quaternion.N_PARAMS, 1);

            final Matrix omegaSkew = new Matrix(
                    Quaternion.N_PARAMS, Quaternion.N_PARAMS);

            // First Runge-Kutta coefficient
            computeOmegaSkew(omega0, omegaSkew);
            omegaSkew.multiply(quat, k1);
            k1.multiplyByScalar(0.5);

            // Second Runge-Kutta coefficient
            tmpQ.copyFrom(k1);
            tmpQ.multiplyByScalar(0.5 * dt);
            tmpQ.add(quat);
            computeOmegaSkew(omega01, omegaSkew);
            omegaSkew.multiply(tmpQ, k2);
            k2.multiplyByScalar(0.5);

            // Third Runge-Kutta coefficient (same omega skew as second coeff.)
            tmpQ.copyFrom(k2);
            tmpQ.multiplyByScalar(0.5 * dt);
            tmpQ.add(quat);
            omegaSkew.multiply(tmpQ, k3);
            k3.multiplyByScalar(0.5);

            // Forth Runge-Kutta coefficient
            tmpQ.copyFrom(k3);
            tmpQ.multiplyByScalar(dt);
            tmpQ.add(quat);
            computeOmegaSkew(omega1, omegaSkew);
            omegaSkew.multiply(tmpQ, k4);
            k4.multiplyByScalar(0.5);

            final double mult1 = 1.0 / 6.0;
            final double mult2 = 1.0 / 3.0;

            // result = quat + dt * (mult1 * k1 + mult2 * k2 + mult2 * k3 + mult1 * k4)
            k1.multiplyByScalar(mult1);
            k2.multiplyByScalar(mult2);
            k3.multiplyByScalar(mult2);
            k4.multiplyByScalar(mult1);

            result.copyFrom(k1);
            result.add(k2);
            result.add(k3);
            result.add(k4);
            result.multiplyByScalar(dt);
            result.add(quat);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        normalizeQuaternion(result);
    }

    /**
     * Performs a RK4 Runge-Kutta integration step.
     *
     * @param quaternion the initial rotation.
     * @param omega0     initial rotation velocity at time t0 expressed in radians
     *                   per second (rad/s). Must have length 3.
     * @param omega1     final rotation velocity at time t1 expressed in radians
     *                   per second (rad/s). Must have length 3.
     * @param dt         time step expressed in seconds (t1 - t0).
     * @param result     resulting final rotation.
     */
    private static void quatIntegrationStepRK4(
            final Quaternion quaternion,
            final double[] omega0,
            final double[] omega1,
            final double dt,
            final Quaternion result) {
        try {
            final Matrix quat = new Matrix(
                    Quaternion.N_PARAMS, 1);
            quaternion.values(quat.getBuffer());

            final Matrix omega0b = Matrix.newFromArray(omega0);
            final Matrix omega1b = Matrix.newFromArray(omega1);

            final Matrix quatResult = new Matrix(
                    Quaternion.N_PARAMS, 1);

            quatIntegrationStepRK4(quat, omega0b, omega1b, dt, quatResult);
            result.setValues(quatResult.getBuffer());

        } catch (WrongSizeException ignore) {
            // never happens
        }
    }
}
