package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.LockedException;

import java.util.Collection;

/**
 * Interface defining gyroscope calibrators.
 *
 * @param <T> a {@link FrameBodyKinematics} containing measures used by the calibrator.
 * @param <L> a listener type.
 */
public interface KnownBiasAndFrameGyroscopeCalibrator<T extends FrameBodyKinematics,
        L extends KnownBiasAndFrameGyroscopeCalibratorListener<?>> extends KnownBiasGyroscopeCalibrator,
        GyroscopeCalibrator {

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
    Collection<? extends T> getMeasurements();

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
    void setMeasurements(final Collection<? extends T> measurements)
            throws LockedException;

    /**
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    L getListener();

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    void setListener(final L listener) throws LockedException;
}
