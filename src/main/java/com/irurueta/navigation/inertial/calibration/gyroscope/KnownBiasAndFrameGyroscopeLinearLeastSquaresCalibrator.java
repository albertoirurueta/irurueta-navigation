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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;

/**
 * Estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * This estimator assumes that biases are known.
 * <p>
 * This calibrator uses a linear approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 6 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 6 different positions, orientations and velocities
 * (although typically velocities are always zero).
 * <p>
 * Measured gyroscope angular rates is assumed to follow the model shown below:
 * <pre>
 *     Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w
 * </pre>
 * Where:
 * - Ωmeas is the measured gyroscope angular rates. This is a 3x1 vector.
 * - bg is the gyroscope bias. This is a known 3x1 vector.
 * - I is the 3x3 identity matrix.
 * - Mg is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect gyroscope, this should be a 3x3 zero matrix.
 * - Ωtrue is ground-truth gyroscope angular rates.
 * - Gg is the G-dependent cross biases introduced by the specific forces sensed
 * by the accelerometer. Ideally, on a perfect gyroscope, this should be a 3x3
 * zero matrix.
 * - ftrue is ground-truth specific force. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public class KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator implements
        KnownBiasAndFrameGyroscopeCalibrator<FrameBodyKinematics,
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener> {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 6;

    /**
     * Number of equations generated for each measurement.
     */
    private static final int EQUATIONS_PER_MEASUREMENT = 3;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 15;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 18;

    /**
     * Contains a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     */
    private Collection<? extends FrameBodyKinematics> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener mListener;

    /**
     * Known x coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double mBiasX;

    /**
     * Known y coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double mBiasY;

    /**
     * Known z coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double mBiasZ;

    /**
     * Estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix mEstimatedGg;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collections of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Gets a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<? extends FrameBodyKinematics> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate the a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @param measurements collection of body kinematics measurements taken at different
     *                     frames (positions, orientations and velocities).
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(
            final Collection<? extends FrameBodyKinematics> measurements)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer and gyroscope,
     * false otherwise.
     */
    @Override
    public boolean isCommonAxisUsed() {
        return mCommonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for accelerometer
     *                       and gyroscope, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setCommonAxisUsed(final boolean commonAxisUsed) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    @Override
    public KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known x coordinate of gyroscope bias.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final AngularSpeed biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAngularSpeed(biasX);
    }

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known y coordinate of gyroscope bias.
     *
     * @param biasY y coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final AngularSpeed biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasY = convertAngularSpeed(biasY);
    }

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known z coordinate of gyroscope bias.
     *
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Sets known gyroscope bias coordinates expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final double biasX, final double biasY, final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Sets known gyroscope bias coordinates.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAngularSpeed(biasX);
        mBiasY = convertAngularSpeed(biasY);
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @return known gyroscope bias.
     */
    public AngularSpeedTriad getBiasAsTriad() {
        return new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND,
                mBiasX, mBiasY, mBiasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(mBiasX, mBiasY, mBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known gyroscope bias.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final AngularSpeedTriad bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAngularSpeed(bias.getValueX(), bias.getUnit());
        mBiasY = convertAngularSpeed(bias.getValueY(), bias.getUnit());
        mBiasZ = convertAngularSpeed(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinate of known bias.
     */
    @Override
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void getBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mBiasX;
        result[1] = mBiasY;
        result[2] = mBiasZ;
    }

    /**
     * Sets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known gyroscope bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setBias(final double[] bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mBiasX = bias[0];
        mBiasY = bias[1];
        mBiasZ = bias[2];
    }

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @return known gyroscope bias as a column matrix.
     */
    @Override
    public Matrix getBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mBiasX);
        result.setElementAtIndex(1, mBiasY);
        result.setElementAtIndex(2, mBiasZ);
    }

    /**
     * Sets known gyroscope bias as a column matrix.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setBias(final Matrix bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS
                || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mBiasX = bias.getElementAtIndex(0);
        mBiasY = bias.getElementAtIndex(1);
        mBiasZ = bias.getElementAtIndex(2);
    }

    /**
     * Indicates whether calibrator is ready to start the calibration.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null
                && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and g-dependant cross biases.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if calibration fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            if (mCommonAxisUsed) {
                calibrateCommonAxis();
            } else {
                calibrateGeneral();
            }

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Gets estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated gyroscope scale factors and cross coupling errors.
     */
    @Override
    public Matrix getEstimatedMg() {
        return mEstimatedMg;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    @Override
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateCommonAxis() throws AlgebraException {
        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // where myx = mzx = mzy = 0

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [0     sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [0     0      sz ]  [Ωtruez]   [g31   g32   g33][ftruez]


        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0     1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0     0      1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23, g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruez  0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                         [mxy]
        //                                                                                                                         [mxz]
        //                                                                                                                         [myz]
        //                                                                                                                         [g11]
        //                                                                                                                         [g12]
        //                                                                                                                         [g13]
        //                                                                                                                         [g21]
        //                                                                                                                         [g22]
        //                                                                                                                         [g23]
        //                                                                                                                         [g31]
        //                                                                                                                         [g32]
        //                                                                                                                         [g33]

        final BodyKinematics expectedKinematics = new BodyKinematics();

        final int rows = EQUATIONS_PER_MEASUREMENT * mMeasurements.size();
        final Matrix a = new Matrix(rows, COMMON_Z_AXIS_UNKNOWNS);
        final Matrix b = new Matrix(rows, 1);
        int i = 0;
        for (final FrameBodyKinematics measurement : mMeasurements) {
            final BodyKinematics measuredKinematics = measurement.getKinematics();
            final ECEFFrame ecefFrame = measurement.getFrame();
            final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
            final double timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame,
                    previousEcefFrame, expectedKinematics);

            final double omegaMeasX = measuredKinematics.getAngularRateX();
            final double omegaMeasY = measuredKinematics.getAngularRateY();
            final double omegaMeasZ = measuredKinematics.getAngularRateZ();

            final double omegaTrueX = expectedKinematics.getAngularRateX();
            final double omegaTrueY = expectedKinematics.getAngularRateY();
            final double omegaTrueZ = expectedKinematics.getAngularRateZ();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, omegaTrueX);
            a.setElementAt(i, 3, omegaTrueY);
            a.setElementAt(i, 4, omegaTrueZ);
            a.setElementAt(i, 6, fTrueX);
            a.setElementAt(i, 7, fTrueY);
            a.setElementAt(i, 8, fTrueZ);

            b.setElementAtIndex(i, omegaMeasX - omegaTrueX - mBiasX);
            i++;

            a.setElementAt(i, 1, omegaTrueY);
            a.setElementAt(i, 5, omegaTrueZ);
            a.setElementAt(i, 9, fTrueX);
            a.setElementAt(i, 10, fTrueY);
            a.setElementAt(i, 11, fTrueZ);

            b.setElementAtIndex(i, omegaMeasY - omegaTrueY - mBiasY);
            i++;

            a.setElementAt(i, 2, omegaTrueZ);
            a.setElementAt(i, 12, fTrueX);
            a.setElementAt(i, 13, fTrueY);
            a.setElementAt(i, 14, fTrueZ);

            b.setElementAtIndex(i, omegaMeasZ - omegaTrueZ - mBiasZ);
            i++;
        }

        final Matrix unknowns = Utils.solve(a, b);

        final double sx = unknowns.getElementAtIndex(0);
        final double sy = unknowns.getElementAtIndex(1);
        final double sz = unknowns.getElementAtIndex(2);
        final double mxy = unknowns.getElementAtIndex(3);
        final double mxz = unknowns.getElementAtIndex(4);
        final double myz = unknowns.getElementAtIndex(5);
        final double g11 = unknowns.getElementAtIndex(6);
        final double g12 = unknowns.getElementAtIndex(7);
        final double g13 = unknowns.getElementAtIndex(8);
        final double g21 = unknowns.getElementAtIndex(9);
        final double g22 = unknowns.getElementAtIndex(10);
        final double g23 = unknowns.getElementAtIndex(11);
        final double g31 = unknowns.getElementAtIndex(12);
        final double g32 = unknowns.getElementAtIndex(13);
        final double g33 = unknowns.getElementAtIndex(14);

        fillMg(sx, sy, sz, mxy, mxz, 0.0, myz, 0.0, 0.0);
        fillGg(g11, g12, g13, g21, g22, g23, g31, g32, g33);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateGeneral() throws AlgebraException {
        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [myx   1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [mzx   mzy    1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
        // g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = myx * Ωtruex + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = mzx * Ωtruex + mzy * Ωtruey + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       0       0       0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruex  Ωtruez  0       0       0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       Ωtruex  Ωtruey  0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                                                 [mxy]
        //                                                                                                                                                 [mxz]
        //                                                                                                                                                 [myx]
        //                                                                                                                                                 [myz]
        //                                                                                                                                                 [mzx]
        //                                                                                                                                                 [mzy]
        //                                                                                                                                                 [g11]
        //                                                                                                                                                 [g12]
        //                                                                                                                                                 [g13]
        //                                                                                                                                                 [g21]
        //                                                                                                                                                 [g22]
        //                                                                                                                                                 [g23]
        //                                                                                                                                                 [g31]
        //                                                                                                                                                 [g32]
        //                                                                                                                                                 [g33]

        final BodyKinematics expectedKinematics = new BodyKinematics();

        final int rows = EQUATIONS_PER_MEASUREMENT * mMeasurements.size();
        final Matrix a = new Matrix(rows, GENERAL_UNKNOWNS);
        final Matrix b = new Matrix(rows, 1);
        int i = 0;
        for (final FrameBodyKinematics measurement : mMeasurements) {
            final BodyKinematics measuredKinematics = measurement.getKinematics();
            final ECEFFrame ecefFrame = measurement.getFrame();
            final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
            final double timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame,
                    previousEcefFrame, expectedKinematics);

            final double omegaMeasX = measuredKinematics.getAngularRateX();
            final double omegaMeasY = measuredKinematics.getAngularRateY();
            final double omegaMeasZ = measuredKinematics.getAngularRateZ();

            final double omegaTrueX = expectedKinematics.getAngularRateX();
            final double omegaTrueY = expectedKinematics.getAngularRateY();
            final double omegaTrueZ = expectedKinematics.getAngularRateZ();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, omegaTrueX);
            a.setElementAt(i, 3, omegaTrueY);
            a.setElementAt(i, 4, omegaTrueZ);
            a.setElementAt(i, 9, fTrueX);
            a.setElementAt(i, 10, fTrueY);
            a.setElementAt(i, 11, fTrueZ);

            b.setElementAtIndex(i, omegaMeasX - omegaTrueX - mBiasX);
            i++;

            a.setElementAt(i, 1, omegaTrueY);
            a.setElementAt(i, 5, omegaTrueX);
            a.setElementAt(i, 6, omegaTrueZ);
            a.setElementAt(i, 12, fTrueX);
            a.setElementAt(i, 13, fTrueY);
            a.setElementAt(i, 14, fTrueZ);

            b.setElementAtIndex(i, omegaMeasY - omegaTrueY - mBiasY);
            i++;

            a.setElementAt(i, 2, omegaTrueZ);
            a.setElementAt(i, 7, omegaTrueX);
            a.setElementAt(i, 8, omegaTrueY);
            a.setElementAt(i, 15, fTrueX);
            a.setElementAt(i, 16, fTrueY);
            a.setElementAt(i, 17, fTrueZ);

            b.setElementAtIndex(i, omegaMeasZ - omegaTrueZ - mBiasZ);
            i++;
        }

        final Matrix unknowns = Utils.solve(a, b);

        final double sx = unknowns.getElementAtIndex(0);
        final double sy = unknowns.getElementAtIndex(1);
        final double sz = unknowns.getElementAtIndex(2);
        final double mxy = unknowns.getElementAtIndex(3);
        final double mxz = unknowns.getElementAtIndex(4);
        final double myx = unknowns.getElementAtIndex(5);
        final double myz = unknowns.getElementAtIndex(6);
        final double mzx = unknowns.getElementAtIndex(7);
        final double mzy = unknowns.getElementAtIndex(8);
        final double g11 = unknowns.getElementAtIndex(9);
        final double g12 = unknowns.getElementAtIndex(10);
        final double g13 = unknowns.getElementAtIndex(11);
        final double g21 = unknowns.getElementAtIndex(12);
        final double g22 = unknowns.getElementAtIndex(13);
        final double g23 = unknowns.getElementAtIndex(14);
        final double g31 = unknowns.getElementAtIndex(15);
        final double g32 = unknowns.getElementAtIndex(16);
        final double g33 = unknowns.getElementAtIndex(17);

        fillMg(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
        fillGg(g11, g12, g13, g21, g22, g23, g31, g32, g33);
    }

    /**
     * Fills scale factor and cross coupling error matrix with estimated values.
     *
     * @param sx  x scale factor
     * @param sy  y scale factor
     * @param sz  z scale factor
     * @param mxy x-y cross coupling
     * @param mxz x-z cross coupling
     * @param myx y-x cross coupling
     * @param myz y-z cross coupling
     * @param mzx z-x cross coupling
     * @param mzy z-y cross coupling
     * @throws WrongSizeException never happens.
     */
    private void fillMg(final double sx, final double sy, final double sz,
                        final double mxy, final double mxz, final double myx,
                        final double myz, final double mzx, final double mzy)
            throws WrongSizeException {
        if (mEstimatedMg == null) {
            mEstimatedMg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        }

        mEstimatedMg.setElementAt(0, 0, sx);
        mEstimatedMg.setElementAt(1, 0, myx);
        mEstimatedMg.setElementAt(2, 0, mzx);

        mEstimatedMg.setElementAt(0, 1, mxy);
        mEstimatedMg.setElementAt(1, 1, sy);
        mEstimatedMg.setElementAt(2, 1, mzy);

        mEstimatedMg.setElementAt(0, 2, mxz);
        mEstimatedMg.setElementAt(1, 2, myz);
        mEstimatedMg.setElementAt(2, 2, sz);
    }

    /**
     * Fills G-dependant cross biases.
     *
     * @param g11 element 1,1 of G-dependant cross biases matrix.
     * @param g12 element 1,2 of G-dependant cross biases matrix.
     * @param g13 element 1,3 of G-dependant cross biases matrix.
     * @param g21 element 2,1 of G-dependant cross biases matrix.
     * @param g22 element 2,2 of G-dependant cross biases matrix.
     * @param g23 element 2,3 of G-dependant cross biases matrix.
     * @param g31 element 3,1 of G-dependant cross biases matrix.
     * @param g32 element 3,2 of G-dependant cross biases matrix.
     * @param g33 element 3,3 of G-dependant cross biases matrix.
     * @throws WrongSizeException never happens.
     */
    private void fillGg(final double g11, final double g12, final double g13,
                        final double g21, final double g22, final double g23,
                        final double g31, final double g32, final double g33)
            throws WrongSizeException {
        if (mEstimatedGg == null) {
            mEstimatedGg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        }

        mEstimatedGg.setElementAt(0, 0, g11);
        mEstimatedGg.setElementAt(0, 1, g12);
        mEstimatedGg.setElementAt(0, 2, g13);

        mEstimatedGg.setElementAt(1, 0, g21);
        mEstimatedGg.setElementAt(1, 1, g22);
        mEstimatedGg.setElementAt(1, 2, g23);

        mEstimatedGg.setElementAt(2, 0, g31);
        mEstimatedGg.setElementAt(2, 1, g32);
        mEstimatedGg.setElementAt(2, 2, g33);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param value angular speed value.
     * @param unit unit of angular speed value.
     * @return converted value.
     */
    private static double convertAngularSpeed(final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit());
    }
}
