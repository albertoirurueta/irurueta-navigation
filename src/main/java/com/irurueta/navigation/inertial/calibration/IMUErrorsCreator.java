package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.NotReadyException;

/**
 * Utility class to create {@link IMUErrors} by combining different sources of estimated data.
 * Sources of data can be any accelerometer calibrator implementing {@link AccelerometerCalibrationSource},
 * any gyroscope calibrator implementing {@link GyroscopeCalibrationSource}, any measurement generator
 * implementing {@link AccelerometerNoiseRootPsdSource} or {@link GyroscopeNoiseRootPsdSource}.
 */
public class IMUErrorsCreator {

    /**
     * A source of estimated accelerometer calibration parameters.
     */
    private AccelerometerCalibrationSource mAccelerometerCalibrationSource;

    /**
     * A source of estimated gyroscope calibration parameters.
     */
    private GyroscopeCalibrationSource mGyroscopeCalibrationSource;

    /**
     * A source of estimated accelerometer noise root PSD.
     */
    private AccelerometerNoiseRootPsdSource mAccelerometerNoiseRootPsdSource;

    /**
     * A source of estimated gyroscope noise root PSD.
     */
    private GyroscopeNoiseRootPsdSource mGyroscopeNoiseRootPsdSource;

    /**
     * Accelerometer quantization level expressed in meters per squared second (m/s^2).
     * By default it is zero when no quantization is assumed.
     */
    private double mAccelerometerQuantizationLevel;

    /**
     * Gyro quantization level expressed in radians per second (rad/s).
     * By default it is zero when no quantization is assumed.
     */
    private double mGyroQuantizationLevel;

    /**
     * Constructor.
     */
    public IMUErrorsCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrationSource  A source of estimated accelerometer calibration parameters.
     * @param gyroscopeCalibrationSource      A source of estimated gyroscope calibration parameters.
     * @param accelerometerNoiseRootPsdSource A source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource     A source of estimated gyroscope noise root PSD.
     */
    public IMUErrorsCreator(
            final AccelerometerCalibrationSource accelerometerCalibrationSource,
            final GyroscopeCalibrationSource gyroscopeCalibrationSource,
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        mAccelerometerCalibrationSource = accelerometerCalibrationSource;
        mGyroscopeCalibrationSource = gyroscopeCalibrationSource;
        mAccelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
        mGyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrationSource  A source of estimated accelerometer calibration parameters.
     * @param gyroscopeCalibrationSource      A source of estimated gyroscope calibration parameters.
     * @param accelerometerNoiseRootPsdSource A source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource     A source of estimated gyroscope noise root PSD.
     * @param accelerometerQuantizationLevel  Accelerometer quantization level expressed in meters per squared
     *                                        second (m/s^2).
     * @param gyroQuantizationLevel           Gyro quantization level expressed in radians per second (rad/s).
     */
    public IMUErrorsCreator(
            final AccelerometerCalibrationSource accelerometerCalibrationSource,
            final GyroscopeCalibrationSource gyroscopeCalibrationSource,
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource,
            final double accelerometerQuantizationLevel,
            final double gyroQuantizationLevel) {
        this(accelerometerCalibrationSource, gyroscopeCalibrationSource,
                accelerometerNoiseRootPsdSource, gyroscopeNoiseRootPsdSource);
        mAccelerometerQuantizationLevel = accelerometerQuantizationLevel;
        mGyroQuantizationLevel = gyroQuantizationLevel;
    }

    /**
     * Gets source of estimated accelerometer calibration parameters.
     *
     * @return source of estimated accelerometer calibration parameters.
     */
    public AccelerometerCalibrationSource getAccelerometerCalibrationSource() {
        return mAccelerometerCalibrationSource;
    }

    /**
     * Sets source of estimated accelerometer calibration parameters.
     *
     * @param accelerometerCalibrationSource source of estimated accelerometer
     *                                       parameters.
     */
    public void setAccelerometerCalibrationSource(
            final AccelerometerCalibrationSource accelerometerCalibrationSource) {
        mAccelerometerCalibrationSource = accelerometerCalibrationSource;
    }

    /**
     * Gets source of estimated gyroscope calibration parameters.
     *
     * @return source of estimated gyroscope calibration parameters.
     */
    public GyroscopeCalibrationSource getGyroscopeCalibrationSource() {
        return mGyroscopeCalibrationSource;
    }

    /**
     * Sets source of estimated gyroscope calibration parameters.
     *
     * @param gyroscopeCalibrationSource source of estimated gyroscope calibration
     *                                   parameters.
     */
    public void setGyroscopeCalibrationSource(
            final GyroscopeCalibrationSource gyroscopeCalibrationSource) {
        mGyroscopeCalibrationSource = gyroscopeCalibrationSource;
    }

    /**
     * Gets source of estimated accelerometer noise root PSD.
     *
     * @return source of estimated accelerometer noise root PSD.
     */
    public AccelerometerNoiseRootPsdSource getAccelerometerNoiseRootPsdSource() {
        return mAccelerometerNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated accelerometer noise root PSD.
     *
     * @param accelerometerNoiseRootPsdSource source of estimated accelerometer
     *                                        noise root PSD.
     */
    public void setAccelerometerNoiseRootPsdSource(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource) {
        mAccelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
    }

    /**
     * Gets source of estimated gyroscope noise root PSD.
     *
     * @return source of estimated gyroscope noise root PSD.
     */
    public GyroscopeNoiseRootPsdSource getGyroscopeNoiseRootPsdSource() {
        return mGyroscopeNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated gyroscope noise root PSD.
     *
     * @param gyroscopeNoiseRootPsdSource source of estimated gyroscope noise
     *                                    root PSD.
     */
    public void sstGyroscopeNoiseRootPsdSource(
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        mGyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Gets accelerometer quantization level expressed in meters per squared second
     * (m/s^2).
     * By default it is zero when no quantization is assumed.
     *
     * @return accelerometer quantization level.
     */
    public double getAccelerometerQuantizationLevel() {
        return mAccelerometerQuantizationLevel;
    }

    /**
     * Sets accelerometer quantization level expressed in meters per squared second
     * (m/s^2).
     *
     * @param accelerometerQuantizationLevel accelerometer quantizaiton level.
     */
    public void setAccelerometerQuantizationLevel(final double accelerometerQuantizationLevel) {
        mAccelerometerQuantizationLevel = accelerometerQuantizationLevel;
    }

    /**
     * Gets gyroscope quantization level expressed in radians per second (rad/s).
     * By default it is zero when no quantization is assumed.
     *
     * @return gyroscope quantization level.
     */
    public double getGyroQuantizationLevel() {
        return mGyroQuantizationLevel;
    }

    /**
     * Sets gyroscope quantization level expressed in radians per second (rad/s).
     *
     * @param gyroQuantizationLevel gyroscope quantization level.
     */
    public void setGyroQuantizationLevel(final double gyroQuantizationLevel) {
        mGyroQuantizationLevel = gyroQuantizationLevel;
    }

    /**
     * Indicates whether all sources have been provided in order to be able
     * to create an {@link IMUErrors} instance.
     *
     * @return true if creator is ready, false otherwise.
     */
    public boolean isReady() {
        return mAccelerometerNoiseRootPsdSource != null && mGyroscopeNoiseRootPsdSource != null
                && mAccelerometerCalibrationSource != null
                && mAccelerometerCalibrationSource.getEstimatedBiases() != null
                && mAccelerometerCalibrationSource.getEstimatedMa() != null
                && mGyroscopeCalibrationSource != null
                && mGyroscopeCalibrationSource.getEstimatedBiases() != null
                && mGyroscopeCalibrationSource.getEstimatedMg() != null
                && mGyroscopeCalibrationSource.getEstimatedGg() != null;
    }

    /**
     * Creates an {@link IMUErrors} instance containing estimated IMU
     * (accelerometer + gyroscope) parameters.
     *
     * @return instance containing estimated IMU parameters.
     * @throws NotReadyException if creator is not ready.
     */
    public IMUErrors create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final double[] ba = mAccelerometerCalibrationSource.getEstimatedBiases();
        final Matrix ma = mAccelerometerCalibrationSource.getEstimatedMa();

        final double[] bg = mGyroscopeCalibrationSource.getEstimatedBiases();
        final Matrix mg = mGyroscopeCalibrationSource.getEstimatedMg();
        final Matrix gg = mGyroscopeCalibrationSource.getEstimatedGg();

        final double accelerometerNoiseRootPsd = mAccelerometerNoiseRootPsdSource
                .getAccelerometerBaseNoiseLevelRootPsd();
        final double gyroscopeNoiseRootPsd = mGyroscopeNoiseRootPsdSource
                .getGyroscopeBaseNoiseLevelRootPsd();

        return new IMUErrors(ba, bg, ma, mg, gg, accelerometerNoiseRootPsd, gyroscopeNoiseRootPsd,
                mAccelerometerQuantizationLevel, mGyroQuantizationLevel);
    }
}
