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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Estimates accelerometer biases, cross couplings and scaling factors.
 * <p>
 * This calibrator uses a linear approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, accelerometer samples must be obtained at 4
 * different positions, orientations and velocities (although typically velocities are
 * always zero).
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias. Ideally, on a perfect accelerometer, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-truth specific force. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public class KnownFrameAccelerometerLinearLeastSquaresCalibrator implements
        KnownFrameAccelerometerCalibrator<FrameBodyKinematics,
                KnownFrameAccelerometerLinearLeastSquaresCalibratorListener> {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 4;

    /**
     * Number of equations generated for each measurement.
     */
    private static final int EQUATIONS_PER_MEASUREMENT = 3;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 12;

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
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameAccelerometerLinearLeastSquaresCalibratorListener mListener;

    /**
     * Estimated accelerometer biases for each IMU axis expressed in meter per squared
     * second (m/s^2).
     */
    private double[] mEstimatedBiases;

    /**
     * Estimated accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMa;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements,
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements,
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
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
    public void setMeasurements(final Collection<? extends FrameBodyKinematics> measurements)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
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
     * When enabled, this eliminates 3 variables from Ma matrix.
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibratorListener getListener() {
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
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
     * and cross-coupling errors.
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
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    @Override
    public double[] getEstimatedBiases() {
        return mEstimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where estimated accelerometer biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    @Override
    public boolean getEstimatedBiases(final double[] result) {
        if (mEstimatedBiases != null) {
            System.arraycopy(mEstimatedBiases, 0, result,
                    0, mEstimatedBiases.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return column matrix containing x,y,z components of estimated accelerometer
     * biases.
     */
    @Override
    public Matrix getEstimatedBiasesAsMatrix() {
        return mEstimatedBiases != null ? Matrix.newFromArray(mEstimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
    public boolean getEstimatedBiasesAsMatrix(final Matrix result)
            throws WrongSizeException {
        if (mEstimatedBiases != null) {
            result.fromArray(mEstimatedBiases);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFx() {
        return mEstimatedBiases != null ? mEstimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFy() {
        return mEstimatedBiases != null ? mEstimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFz() {
        return mEstimatedBiases != null ? mEstimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFxAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[0],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFxAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[0]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFyAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[1],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFyAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[1]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFzAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[2],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFzAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[2]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @return estimated accelerometer bias or null if not available.
     */
    @Override
    public AccelerationTriad getEstimatedBiasAsTriad() {
        return mEstimatedBiases != null ?
                new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                        mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2]) : null;
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated accelerometer bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedBiasAsTriad(final AccelerationTriad result) {
        if (mEstimatedBiases != null) {
            result.setValueCoordinatesAndUnit(
                    mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2],
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated accelerometer scale factors and ross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated accelerometer scale factors and cross coupling errors, or null
     * if not available.
     */
    @Override
    public Matrix getEstimatedMa() {
        return mEstimatedMa;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 1) : null;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateCommonAxis() throws AlgebraException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        // where myx = mzx = mzy = 0

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [0     sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [ftruez]

        //  [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        //  [fmeasy]   [by]     [0      1+sy    myz ][ftruey]
        //  [fmeasz]   [bz]     [0      0       1+sz][ftruez]

        //  fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + (1+sy) * ftruey + myz * ftruez
        //  fmeasz = bz + (1+sz) * ftruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz
        // Reordering:
        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + ftruez + sz * ftruez

        //  fmeasx - ftruex = bx + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy - ftruey = by + sy * ftruey + myz * ftruez
        //  fmeasz - ftruez = bz + sz * ftruez

        // [1   0   0   ftruex  0       0       ftruey  ftruez  0     ][bx ] = [fmeasx - ftruex]
        // [0   1   0   0       ftruey  0       0       0       ftruez][by ]   [fmeasy - ftruey]
        // [0   0   1   0       0       ftruez  0       0       0     ][bz ]   [fmeasz - ftruez]
        //                                                             [sx ]
        //                                                             [sy ]
        //                                                             [sz ]
        //                                                             [mxy]
        //                                                             [mxz]
        //                                                             [myz]

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

            final double fMeasX = measuredKinematics.getFx();
            final double fMeasY = measuredKinematics.getFy();
            final double fMeasZ = measuredKinematics.getFz();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, fTrueX);
            a.setElementAt(i, 6, fTrueY);
            a.setElementAt(i, 7, fTrueZ);

            b.setElementAtIndex(i, fMeasX - fTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, fTrueY);
            a.setElementAt(i, 8, fTrueZ);

            b.setElementAtIndex(i, fMeasY - fTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, fTrueZ);

            b.setElementAtIndex(i, fMeasZ - fTrueZ);
            i++;
        }

        final Matrix unknowns = Utils.solve(a, b);

        final double bx = unknowns.getElementAtIndex(0);
        final double by = unknowns.getElementAtIndex(1);
        final double bz = unknowns.getElementAtIndex(2);
        final double sx = unknowns.getElementAtIndex(3);
        final double sy = unknowns.getElementAtIndex(4);
        final double sz = unknowns.getElementAtIndex(5);
        final double mxy = unknowns.getElementAtIndex(6);
        final double mxz = unknowns.getElementAtIndex(7);
        final double myz = unknowns.getElementAtIndex(8);

        fillBiases(bx, by, bz);
        fillMa(sx, sy, sz, mxy, mxz, 0.0, myz, 0.0, 0.0);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateGeneral() throws AlgebraException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy]   [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        //  [fmeasz]   [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        //  [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        //  [fmeasy]   [by]     [myx    1+sy    myz ][ftruey]
        //  [fmeasz]   [bz]     [mzx    mzy     1+sz][ftruez]

        //  fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + (1+sy) * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + (1+sz) * ftruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        //  fmeasx - ftruex = bx + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy - ftruey = by + myx * ftruex + sy * ftruey + myz * ftruez
        //  fmeasz - ftruez = bz + mzx * ftruex + mzy * ftruey + sz * ftruez

        // [1   0   0   ftruex  0       0       ftruey  ftruez  0       0       0       0     ][bx ] = [fmeasx - ftruex]
        // [0   1   0   0       ftruey  0       0       0       ftruex  ftruez  0       0     ][by ]   [fmeasy - ftruey]
        // [0   0   1   0       0       ftruez  0       0       0       0       ftruex  ftruey][bz ]   [fmeasz - ftruez]
        //                                                                                     [sx ]
        //                                                                                     [sy ]
        //                                                                                     [sz ]
        //                                                                                     [mxy]
        //                                                                                     [mxz]
        //                                                                                     [myx]
        //                                                                                     [myz]
        //                                                                                     [mzx]
        //                                                                                     [mzy]

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

            final double fMeasX = measuredKinematics.getFx();
            final double fMeasY = measuredKinematics.getFy();
            final double fMeasZ = measuredKinematics.getFz();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, fTrueX);
            a.setElementAt(i, 6, fTrueY);
            a.setElementAt(i, 7, fTrueZ);

            b.setElementAtIndex(i, fMeasX - fTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, fTrueY);
            a.setElementAt(i, 8, fTrueX);
            a.setElementAt(i, 9, fTrueZ);

            b.setElementAtIndex(i, fMeasY - fTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, fTrueZ);
            a.setElementAt(i, 10, fTrueX);
            a.setElementAt(i, 11, fTrueY);

            b.setElementAtIndex(i, fMeasZ - fTrueZ);
            i++;
        }

        final Matrix unknowns = Utils.solve(a, b);

        final double bx = unknowns.getElementAtIndex(0);
        final double by = unknowns.getElementAtIndex(1);
        final double bz = unknowns.getElementAtIndex(2);
        final double sx = unknowns.getElementAtIndex(3);
        final double sy = unknowns.getElementAtIndex(4);
        final double sz = unknowns.getElementAtIndex(5);
        final double mxy = unknowns.getElementAtIndex(6);
        final double mxz = unknowns.getElementAtIndex(7);
        final double myx = unknowns.getElementAtIndex(8);
        final double myz = unknowns.getElementAtIndex(9);
        final double mzx = unknowns.getElementAtIndex(10);
        final double mzy = unknowns.getElementAtIndex(11);

        fillBiases(bx, by, bz);
        fillMa(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Fills estimated biases array with estimated values.
     *
     * @param bx x coordinate of bias.
     * @param by y coordinate of bias.
     * @param bz z coordinate of bias.
     */
    private void fillBiases(final double bx, final double by, final double bz) {
        if (mEstimatedBiases == null) {
            mEstimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        mEstimatedBiases[0] = bx;
        mEstimatedBiases[1] = by;
        mEstimatedBiases[2] = bz;
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
    private void fillMa(final double sx, final double sy, final double sz,
                        final double mxy, final double mxz, final double myx,
                        final double myz, final double mzx, final double mzy)
            throws WrongSizeException {
        if (mEstimatedMa == null) {
            mEstimatedMa = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        }

        mEstimatedMa.setElementAt(0, 0, sx);
        mEstimatedMa.setElementAt(1, 0, myx);
        mEstimatedMa.setElementAt(2, 0, mzx);

        mEstimatedMa.setElementAt(0, 1, mxy);
        mEstimatedMa.setElementAt(1, 1, sy);
        mEstimatedMa.setElementAt(2, 1, mzy);

        mEstimatedMa.setElementAt(0, 2, mxz);
        mEstimatedMa.setElementAt(1, 2, myz);
        mEstimatedMa.setElementAt(2, 2, sz);
    }
}
