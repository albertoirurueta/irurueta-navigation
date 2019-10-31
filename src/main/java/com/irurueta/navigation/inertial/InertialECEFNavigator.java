package com.irurueta.navigation.inertial;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameException;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.estimators.GravityEstimator;
import com.irurueta.units.*;

/**
 * Runs precision ECEF-frame inertial navigation equations.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/InertialDemoECEF
 */
@SuppressWarnings("WeakerAccess")
public class InertialECEFNavigator {

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Alpha threshold.
     */
    private static final double ALPHA_THRESHOLD = 1e-8;

    /**
     * Number of rows.
     */
    private static final int ROWS = 3;

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECEFFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECEFFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECEFFrame oldFrame,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECEFFrame oldFrame,
                         final Kinematics kinematics,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECEFFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECEFFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECEFFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECEFFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECEFFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECEFFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECEFFrame result) throws InertialNavigatorException {
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Point3D oldPosition,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy,
                oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Point3D oldPosition,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy,
                oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Point3D oldPosition,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Point3D oldPosition,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, oldVx,
                oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx,
                oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final double oldX,
                                          final double oldY,
                                          final double oldZ,
                                          final CoordinateTransformation oldC,
                                          final double oldVx,
                                          final double oldVy,
                                          final double oldVz,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final Distance oldX,
                                          final Distance oldY,
                                          final Distance oldZ,
                                          final CoordinateTransformation oldC,
                                          final Speed oldSpeedX,
                                          final Speed oldSpeedY,
                                          final Speed oldSpeedZ,
                                          final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final ECEFFrame oldFrame,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final ECEFFrame oldFrame,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Kinematics kinematics)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Kinematics kinematics)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, kinematics);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final double angularRateX,
                                          final double angularRateY,
                                          final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final ECEFFrame oldFrame,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final ECEFFrame oldFrame,
                                          final double fx,
                                          final double fy,
                                          final double fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final double timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECEFFrame navigateAndReturnNew(final Time timeInterval,
                                          final ECEFFrame oldFrame,
                                          final Acceleration fx,
                                          final Acceleration fy,
                                          final Acceleration fz,
                                          final AngularSpeed angularRateX,
                                          final AngularSpeed angularRateY,
                                          final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToEcefCoordinateTransformationMatrix(oldC)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        try {
            // Attitude update
            // From (2.145) determine the Earth rotation over the update interval
            final double alpha = EARTH_ROTATION_RATE * timeInterval;
            final Matrix cEarth = CoordinateTransformation.ecefToEciMatrixFromAngle(alpha);

            // Calculate attidue increment, magnitude, and skew-symmetric matrix
            final double alphaX = angularRateX * timeInterval;
            final double alphaY = angularRateY * timeInterval;
            final double alphaZ = angularRateZ * timeInterval;

            final double alphaNorm = Math.sqrt(alphaX * alphaX
                    + alphaY * alphaY + alphaZ * alphaZ);
            final Matrix alphaSkew1 = Utils.skewMatrix(
                    new double[]{alphaX, alphaY, alphaZ});

            // Obtain coordinate transformation matrix from the new attitude with
            // respect an inertial frame to the old using Rodrigues' formula, (5.73)
            final Matrix cNewOld = Matrix.identity(ROWS, ROWS);
            if (alphaNorm > ALPHA_THRESHOLD) {
                final double alphaNorm2 = alphaNorm * alphaNorm;
                final double value1 = Math.sin(alphaNorm) / alphaNorm;
                final double value2 = (1.0 - Math.cos(alphaNorm)) / alphaNorm2;
                final Matrix tmp1 = alphaSkew1.multiplyByScalarAndReturnNew(value1);
                final Matrix tmp2 = alphaSkew1.multiplyByScalarAndReturnNew(value2);
                tmp2.multiply(alphaSkew1);

                cNewOld.add(tmp1);
                cNewOld.add(tmp2);
            } else {
                cNewOld.add(alphaSkew1);
            }

            // Update attitude using (5.75)
            Matrix oldCbe = oldC.getMatrix();
            cEarth.multiply(oldCbe);
            cEarth.multiply(cNewOld); // cbe = cEarth * oldCbe * cNewOld

            // Specific force frame transformation
            // Calculate the average body-to-ECEF-frame coordinate transformation
            // matrix over the update interval using (5.84) and (5.85).
            if (alphaNorm > ALPHA_THRESHOLD) {
                final double alphaNorm2 = alphaNorm * alphaNorm;
                final double value1 = (1.0 - Math.cos(alphaNorm)) / alphaNorm2;
                final double value2 = (1.0 - Math.sin(alphaNorm) / alphaNorm)
                        / alphaNorm2;
                final Matrix tmp1 = alphaSkew1.multiplyByScalarAndReturnNew(value1);
                final Matrix tmp2 = alphaSkew1.multiplyByScalarAndReturnNew(value2);
                tmp2.multiply(alphaSkew1);

                final Matrix tmp3 = Matrix.identity(ROWS, ROWS);
                tmp3.add(tmp1);
                tmp3.add(tmp2);

                oldCbe.multiply(tmp3);
            }

            final Matrix alphaSkew2 = Utils.skewMatrix(
                    new double[]{0.0, 0.0, alpha});
            alphaSkew2.multiplyByScalar(0.5);
            alphaSkew2.multiply(oldCbe); // 0.5 * alphaSkew2 * oldCbe

            oldCbe.subtract(alphaSkew2); // oldCbe - 0.5 * alphaSkew2 * oldCbe
            // oldCbe now contains the average body-to-ECEF-frame coordinate transformation

            // Transform specific force to ECEF-frame resolving axes using (5.85)
            final Matrix fibb = new Matrix(ROWS, 1);
            fibb.setElementAtIndex(0, fx);
            fibb.setElementAtIndex(1, fy);
            fibb.setElementAtIndex(2, fz);

            // fibe = aveCbe * fibb
//            final Matrix fibe = oldCbe.multiplyAndReturnNew(fibb);
            oldCbe.multiply(fibb);

            // Update velocity
            // From (5.36)
            final Gravity gravity = GravityEstimator.estimateGravityAndReturnNew(oldX, oldY, oldZ);
            final Matrix g = gravity.asMatrix();

            final Matrix oldVebe = new Matrix(ROWS, 1);
            oldVebe.setElementAtIndex(0, oldVx);
            oldVebe.setElementAtIndex(1, oldVy);
            oldVebe.setElementAtIndex(2, oldVz);

            final Matrix skew3 = Utils.skewMatrix(
                    new double[]{0.0, 0.0, EARTH_ROTATION_RATE});
            skew3.multiplyByScalar(2.0); // 2.0 * omegaSkew
            skew3.multiply(oldVebe); // 2.0 * omegaSkew * oldVebe

            oldCbe.add(g); // fibe + g
            oldCbe.subtract(skew3); // fibe + g - 2.0 * omegaSkew * oldVebe

            oldCbe.multiplyByScalar(timeInterval); // timeInterval * (fibe + g - 2.0 * omegaSkew * oldVebe)

            oldVebe.add(oldCbe); // oldVebe + timeInterval * (fibe + g - 2.0 * omegaSkew * oldVebe)

            final double newVx = oldVebe.getElementAtIndex(0);
            final double newVy = oldVebe.getElementAtIndex(1);
            final double newVz = oldVebe.getElementAtIndex(2);

            // Update cartesian position
            // From (5.38)
            final double newX = oldX + (newVx + oldVx) * 0.5 * timeInterval;
            final double newY = oldY + (newVy + oldVy) * 0.5 * timeInterval;
            final double newZ = oldZ + (newVz + oldVz) * 0.5 * timeInterval;

            final CoordinateTransformation newC =
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            newC.setMatrix(cEarth); //cEarth contains cBe

            result.setCoordinateTransformation(newC);
            result.setVelocityCoordinates(newVx, newVy, newVz);
            result.setCoordinates(newX, newY, newZ);

        } catch (final AlgebraException | FrameException | InvalidRotationMatrixException e) {
            throw new InertialNavigatorException(e);
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval),
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Point3D oldPosition,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition.getInhomX(), oldPosition.getInhomY(), oldPosition.getInhomZ(),
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Point3D oldPosition,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition.getInhomX(), oldPosition.getInhomY(), oldPosition.getInhomZ(),
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Point3D oldPosition,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Point3D oldPosition,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                convertAccelerationToDouble(fx), convertAccelerationToDouble(fy),
                convertAccelerationToDouble(fz), angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, convertAccelerationToDouble(fx),
                convertAccelerationToDouble(fy), convertAccelerationToDouble(fz),
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), convertAccelerationToDouble(fx),
                convertAccelerationToDouble(fy), convertAccelerationToDouble(fz),
                convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ),
                result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                convertAccelerationToDouble(fx), convertAccelerationToDouble(fy),
                convertAccelerationToDouble(fz),
                convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final double oldX,
                                    final double oldY,
                                    final double oldZ,
                                    final CoordinateTransformation oldC,
                                    final double oldVx,
                                    final double oldVy,
                                    final double oldVz,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final double timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final Distance oldX,
                                    final Distance oldY,
                                    final Distance oldZ,
                                    final CoordinateTransformation oldC,
                                    final Speed oldSpeedX,
                                    final Speed oldSpeedY,
                                    final Speed oldSpeedZ,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final double timeInterval,
                                    final ECEFFrame oldFrame,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final ECEFFrame oldFrame,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        navigateECEF(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final double timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), kinematics,
                    result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Kinematics kinematics,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        navigateECEF(convertTimeToDouble(timeInterval), oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final double timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final double angularRateX,
                                    final double angularRateY,
                                    final double angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        navigateECEF(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final double timeInterval,
                                    final ECEFFrame oldFrame,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final ECEFFrame oldFrame,
                                    final double fx,
                                    final double fy,
                                    final double fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        navigateECEF(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final double timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECEF frame containing new body
     *                     position, velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECEF(final Time timeInterval,
                                    final ECEFFrame oldFrame,
                                    final Acceleration fx,
                                    final Acceleration fy,
                                    final Acceleration fz,
                                    final AngularSpeed angularRateX,
                                    final AngularSpeed angularRateY,
                                    final AngularSpeed angularRateZ,
                                    final ECEFFrame result)
            throws InertialNavigatorException {
        navigateECEF(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Point3D oldPosition,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Point3D oldPosition,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Point3D oldPosition,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Point3D oldPosition,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx,
                fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final double oldX,
                                                     final double oldY,
                                                     final double oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final double oldVx,
                                                     final double oldVy,
                                                     final double oldVz,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy,
                fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     * body-to-ECEF-frame coordinate transformation matrix are invalid.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final Distance oldX,
                                                     final Distance oldY,
                                                     final Distance oldZ,
                                                     final CoordinateTransformation oldC,
                                                     final Speed oldSpeedX,
                                                     final Speed oldSpeedY,
                                                     final Speed oldSpeedZ,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Kinematics kinematics)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final double angularRateX,
                                                     final double angularRateY,
                                                     final double angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final double fx,
                                                     final double fy,
                                                     final double fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final double timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECEF-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECEF frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECEF frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECEF frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECEFFrame navigateECEFAndReturnNew(final Time timeInterval,
                                                     final ECEFFrame oldFrame,
                                                     final Acceleration fx,
                                                     final Acceleration fy,
                                                     final Acceleration fz,
                                                     final AngularSpeed angularRateX,
                                                     final AngularSpeed angularRateY,
                                                     final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECEFFrame result = new ECEFFrame();
        navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
        return result;
    }

    /**
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to ECEF transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidBodyToEcefCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return ECEFFrame.isValidCoordinateTransformationMatrix(c);
    }

    /**
     * Converts provided time instance into its corresponding value expressed in
     * seconds.
     *
     * @param time time instance to be converted.
     * @return converted value expressed in seconds.
     */
    private static double convertTimeToDouble(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(),
                TimeUnit.SECOND);
    }

    /**
     * Converts provided distance instance into its corresponding value expressed in
     * meters.
     *
     * @param distance distance instance to be converted.
     * @return converted value expressed in meters.
     */
    private static double convertDistanceToDouble(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Converts provided speed instance into its corresponding value expressed in
     * meters per second.
     *
     * @param speed speed instance to be converted.
     * @return converted value expressed in meters per second.
     */
    private static double convertSpeedToDouble(final Speed speed) {
        return SpeedConverter.convert(speed.getValue().doubleValue(),
                speed.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Converts provided acceleration instance into its corresponding value expressed
     * in meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value expressed in meters per squared second.
     */
    private static double convertAccelerationToDouble(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts provided angular speed into its corresponding value expressed in
     * radians per second.
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value expressed in radians per second.
     */
    private static double convertAngularSpeedToDouble(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }
}
