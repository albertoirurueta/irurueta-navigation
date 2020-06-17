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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;

import java.io.IOException;
import java.util.Collection;

/**
 * Estimates magnetometer hard-iron biases, soft-iron cross couplings and
 * scaling factors.
 * <p>
 * This calibrator uses a linear approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, magnetometer samples must be obtained at 4
 * different positions or orientations.
 * Notice that frame velocities are ignored by this calibrator.
 * <p>
 * Measured magnetic flux density is assumed to follow the model shown below:
 * <pre>
 *     mBmeas = bm + (I + Mm) * mBtrue + w
 * </pre>
 * Where:
 * - mBmeas is the measured magnetic flux density. This is a 3x1 vector.
 * - bm is magnetometer hard-iron bias. Ideally, on a perfect magnetometer,
 * this should be a 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mm is the 3x3 soft-iron matrix containing cross-couplings and scaling
 * factors. Ideally, on a perfect magnetometer, this should be a 3x3 zero
 * matrix.
 * - mBtrue is ground-truth magnetic flux density. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public class KnownFrameMagnetometerLinearLeastSquaresCalibrator implements
        KnownFrameMagnetometerCalibrator<FrameBodyMagneticFluxDensity,
                KnownFrameMagnetometerLinearLeastSquaresCalibratorListener> {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
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
     * Number of unknowns when common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 12;

    /**
     * Contains a collection of body magnetic flux density measurements taken
     * at different frames (positions and orientations).
     * If a single device magnetometer needs to be calibrated, typically all
     * measurements are taken at the same position, with zero velocity and
     * multiple orientations.
     * However, if we just want to calibrate a given magnetometer model (e.g.
     * obtain an average and less precise calibration for the magnetometer of
     * a given phone model), we could take measurements collected throughout
     * the planet at multiple positions while the phone remains static (e.g.
     * while charging), hence each measurement position will change, velocity
     * will remain zero and orientation will be typically constant at
     * horizontal orientation while the phone remains on a
     * flat surface.
     */
    private Collection<? extends FrameBodyMagneticFluxDensity> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameMagnetometerLinearLeastSquaresCalibratorListener mListener;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] mEstimatedHardIron;

    /**
     * Estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMm;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * Constructor.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements
     *                     taken at different frames (positions and orientations).
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements
     *                     taken at different frames (positions and orientations).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements
     *                       taken at different frames (positions and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements
     *                       taken at different frames (positions and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel) {
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements
     *                      taken at different frames (positions and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel) {
        this(measurements);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements
     *                      taken at different frames (positions and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel) {
        this(commonAxisUsed);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements
     *                       taken at different frames (positions and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel) {
        this(measurements, commonAxisUsed);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements
     *                       taken at different frames (positions and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Gets a collection of body magnetic flux density measurements taken at different
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
     * @return a collection of body magnetic flux density measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<? extends FrameBodyMagneticFluxDensity> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a collection of body magnetic flux density measurements taken at different
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
     * @param measurements collection of body magnetic flux density measurements
     *                     taken at different frames (positions, orientations
     *                     and velocities).
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setMeasurements(
            final Collection<? extends FrameBodyMagneticFluxDensity> measurements)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm (soft-iron) matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer, false otherwise.
     */
    @Override
    public boolean isCommonAxisUsed() {
        return mCommonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mm matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for
     *                       accelerometer, gyroscope and magnetometer, false
     *                       otherwise.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setCommonAxisUsed(final boolean commonAxisUsed)
            throws LockedException {
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
    public KnownFrameMagnetometerLinearLeastSquaresCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setListener(
            final KnownFrameMagnetometerLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null
                && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is currently running or no.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    @Override
    public WorldMagneticModel getMagneticModel() {
        return mMagneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     * If not provided a default model will be loaded internally.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMagneticModel(final WorldMagneticModel magneticModel)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
    }

    /**
     * Estimates accelerometer calibration parameters containing scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if calibration fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException,
            CalibrationException {
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

        } catch (final AlgebraException | IOException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    @Override
    public double[] getEstimatedHardIron() {
        return mEstimatedHardIron;
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where estimated magnetometer biases will be
     *               stored.
     * @return true if result instance was updated, false otherwise (when
     * estimation is not yet available).
     */
    @Override
    public boolean getEstimatedHardIron(final double[] result) {
        if (mEstimatedHardIron != null) {
            System.arraycopy(mEstimatedHardIron, 0, result,
                    0, mEstimatedHardIron.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @return column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases.
     */
    @Override
    public Matrix getEstimatedHardIronAsMatrix() {
        return mEstimatedHardIron != null ? Matrix.newFromArray(mEstimatedHardIron) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
    public boolean getEstimatedHardIronAsMatrix(final Matrix result)
            throws WrongSizeException {
        if (mEstimatedHardIron != null) {
            result.fromArray(mEstimatedHardIron);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return x coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronX() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[0] : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return y coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronY() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[1] : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return z coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronZ() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[2] : null;
    }

    /**
     * Gets estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated magnetometer soft-iron scale factors and cross coupling errors,
     * or null if not available.
     */
    @Override
    public Matrix getEstimatedMm() {
        return mEstimatedMm;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 1) : null;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for the accelerometer, gyroscope and magnetometer.
     *
     * @throws AlgebraException if there are numerical errors.
     * @throws IOException      if world magnetic model cannot be loaded.
     */
    private void calibrateCommonAxis() throws AlgebraException, IOException {
        // The magnetometer model is:
        // mBmeas = bm + (I + Mm) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = bm + (I + Mm) * mBtrue

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        // where myx = mzx = mzy = 0

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [0     sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [mBtruez]

        //  [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        //  [mBmeasy]   [by]     [0      1+sy    myz ][mBtruey]
        //  [mBmeasz]   [bz]     [0      0       1+sz][mBtruez]

        //  mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + (1+sy) * mBtruey + myz * mBtruez
        //  mBmeasz = bz + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz
        // Reordering:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mBtruez + sz * mBtruez

        //  mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey = by + sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez = bz + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruez][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0      ][bz ]   [mBmeasz - mBtruez]
        //                                                                   [sx ]
        //                                                                   [sy ]
        //                                                                   [sz ]
        //                                                                   [mxy]
        //                                                                   [mxz]
        //                                                                   [myz]

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final BodyMagneticFluxDensity expectedMagneticFluxDensity = new BodyMagneticFluxDensity();
        final NEDFrame nedFrame = new NEDFrame();
        final NEDMagneticFluxDensity earthB = new NEDMagneticFluxDensity();
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);

        final int rows = EQUATIONS_PER_MEASUREMENT * mMeasurements.size();
        final Matrix a = new Matrix(rows, COMMON_Z_AXIS_UNKNOWNS);
        final Matrix b = new Matrix(rows, 1);
        int i = 0;
        for (final FrameBodyMagneticFluxDensity measurement : mMeasurements) {
            final BodyMagneticFluxDensity measuredMagneticFluxDensity =
                    measurement.getMagneticFluxDensity();

            // estimate Earth magnetic flux density at frame position and
            // timestamp using WMM
            final ECEFFrame ecefFrame = measurement.getFrame();
            ECEFtoNEDFrameConverter.convertECEFtoNED(ecefFrame, nedFrame);

            final double year = measurement.getYear();

            final double latitude = nedFrame.getLatitude();
            final double longitude = nedFrame.getLongitude();
            final double height = nedFrame.getHeight();

            nedFrame.getCoordinateTransformation(cbn);
            cbn.inverse(cnb);

            wmmEstimator.estimate(latitude, longitude, height, year, earthB);

            // estimate expected body mangetic flux density taking into
            // account body attitude (inverse of frame orientation) and
            // estimated Earth magnetic flux density
            BodyMagneticFluxDensityEstimator.estimate(earthB, cnb,
                    expectedMagneticFluxDensity);

            final double bMeasX = measuredMagneticFluxDensity.getBx();
            final double bMeasY = measuredMagneticFluxDensity.getBy();
            final double bMeasZ = measuredMagneticFluxDensity.getBz();

            final double bTrueX = expectedMagneticFluxDensity.getBx();
            final double bTrueY = expectedMagneticFluxDensity.getBy();
            final double bTrueZ = expectedMagneticFluxDensity.getBz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, bTrueX);
            a.setElementAt(i, 6, bTrueY);
            a.setElementAt(i, 7, bTrueZ);

            b.setElementAtIndex(i, bMeasX - bTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, bTrueY);
            a.setElementAt(i, 8, bTrueZ);

            b.setElementAtIndex(i, bMeasY - bTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, bTrueZ);

            b.setElementAtIndex(i, bMeasZ - bTrueZ);
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

        fillHardIronBiases(bx, by, bz);
        fillMm(sx, sy, sz, mxy, mxz, 0.0, myz, 0.0, 0.0);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException if there are numerical errors.
     * @throws IOException      if world magnetic model cannot be loaded.
     */
    private void calibrateGeneral() throws AlgebraException, IOException {
        // The magnetometer model is:
        // mBmeas = bm + (I + Mm) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = bm + (I + Mm) * mBtrue

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        //  [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        //  [mBmeasy]   [by]     [myx    1+sy    myz ][mBtruey]
        //  [mBmeasz]   [bz]     [mzx    mzy     1+sz][mBtruez]

        //  mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + myx * mBtruex + (1+sy) * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

        //  mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey = by + myx * mBtruex + sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez = bz + mzx * mBtruex + mzy * mBtruey + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0        0        0        0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruex  mBtruez  0        0      ][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0        0        mBtruex  mBtruey][bz ]   [mBmeasz - mBtruez]
        //                                                                                              [sx ]
        //                                                                                              [sy ]
        //                                                                                              [sz ]
        //                                                                                              [mxy]
        //                                                                                              [mxz]
        //                                                                                              [myx]
        //                                                                                              [myz]
        //                                                                                              [mzx]
        //                                                                                              [mzy]

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final BodyMagneticFluxDensity expectedMagneticFluxDensity = new BodyMagneticFluxDensity();
        final NEDFrame nedFrame = new NEDFrame();
        final NEDMagneticFluxDensity earthB = new NEDMagneticFluxDensity();
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);

        final int rows = EQUATIONS_PER_MEASUREMENT * mMeasurements.size();
        final Matrix a = new Matrix(rows, GENERAL_UNKNOWNS);
        final Matrix b = new Matrix(rows, 1);
        int i = 0;
        for (final FrameBodyMagneticFluxDensity measurement : mMeasurements) {
            final BodyMagneticFluxDensity measuredMagneticFluxDensity =
                    measurement.getMagneticFluxDensity();

            // estimate Earth magnetic flux density at frame position and
            // timestamp using WMM
            final ECEFFrame ecefFrame = measurement.getFrame();
            ECEFtoNEDFrameConverter.convertECEFtoNED(ecefFrame, nedFrame);

            final double year = measurement.getYear();

            final double latitude = nedFrame.getLatitude();
            final double longitude = nedFrame.getLongitude();
            final double height = nedFrame.getHeight();

            nedFrame.getCoordinateTransformation(cbn);
            cbn.inverse(cnb);

            wmmEstimator.estimate(latitude, longitude, height, year, earthB);

            // estimate expected body mangetic flux density taking into
            // account body attitude (inverse of frame orientation) and
            // estimated Earth magnetic flux density
            BodyMagneticFluxDensityEstimator.estimate(earthB, cnb,
                    expectedMagneticFluxDensity);

            final double bMeasX = measuredMagneticFluxDensity.getBx();
            final double bMeasY = measuredMagneticFluxDensity.getBy();
            final double bMeasZ = measuredMagneticFluxDensity.getBz();

            final double bTrueX = expectedMagneticFluxDensity.getBx();
            final double bTrueY = expectedMagneticFluxDensity.getBy();
            final double bTrueZ = expectedMagneticFluxDensity.getBz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, bTrueX);
            a.setElementAt(i, 6, bTrueY);
            a.setElementAt(i, 7, bTrueZ);

            b.setElementAtIndex(i, bMeasX - bTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, bTrueY);
            a.setElementAt(i, 8, bTrueX);
            a.setElementAt(i, 9, bTrueZ);

            b.setElementAtIndex(i, bMeasY - bTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, bTrueZ);
            a.setElementAt(i, 10, bTrueX);
            a.setElementAt(i, 11, bTrueY);

            b.setElementAtIndex(i, bMeasZ - bTrueZ);
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

        fillHardIronBiases(bx, by, bz);
        fillMm(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Fills estimated biases array with estimated values.
     *
     * @param bx x coordinate of bias.
     * @param by y coordinate of bias.
     * @param bz z coordinate of bias.
     */
    private void fillHardIronBiases(
            final double bx, final double by, final double bz) {
        if (mEstimatedHardIron == null) {
            mEstimatedHardIron = new double[BodyMagneticFluxDensity.COMPONENTS];
        }

        mEstimatedHardIron[0] = bx;
        mEstimatedHardIron[1] = by;
        mEstimatedHardIron[2] = bz;
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
    private void fillMm(final double sx, final double sy, final double sz,
                        final double mxy, final double mxz, final double myx,
                        final double myz, final double mzx, final double mzy)
            throws WrongSizeException {
        if (mEstimatedMm == null) {
            mEstimatedMm = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
        }

        mEstimatedMm.setElementAt(0, 0, sx);
        mEstimatedMm.setElementAt(1, 0, myx);
        mEstimatedMm.setElementAt(2, 0, mzx);

        mEstimatedMm.setElementAt(0, 1, mxy);
        mEstimatedMm.setElementAt(1, 1, sy);
        mEstimatedMm.setElementAt(2, 1, mzy);

        mEstimatedMm.setElementAt(0, 2, mxz);
        mEstimatedMm.setElementAt(1, 2, myz);
        mEstimatedMm.setElementAt(2, 2, sz);
    }
}
