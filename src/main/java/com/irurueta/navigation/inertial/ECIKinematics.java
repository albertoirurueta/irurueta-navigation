package com.irurueta.navigation.inertial;

import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;

/**
 * Describes the motion of a body based on the specific forces (i.e. specific accelerations) and
 * angular rates applied to it with respect the ECI frame and resolved along body-frame axes.
 * Body frame axes are typically defined so that x is the forward axis, pointing in the usual direction
 * of travel, z is the down axis, pointing in the usual direction of gravity, and y is the right axis,
 * completing the orthogonal set.
 */
@SuppressWarnings("WeakerAccess")
public class ECIKinematics extends ECIorECEFKinematics<ECIKinematics> {

    /**
     * Constructor.
     */
    public ECIKinematics() {
        super();
    }

    /**
     * Constructor.
     *
     * @param fx Specific force of body frame with respect ECI frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect ECI frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect ECI frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public ECIKinematics(final double fx, final double fy, final double fz) {
        super(fx, fy, fz);
    }

    /**
     * Constructor.
     *
     * @param fx           Specific force of body frame with respect ECI frame resolved along body-frame x-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy           Specific force of body frame with respect ECI frame resolved along body-frame y-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz           Specific force of body frame with respect ECI frame resolved along body-frame z-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param angularRateX Angular rate of body frame with respect ECI frame, resolved about body-frame
     *                     x-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateY Angular rate of body frame with respect ECI frame, resolved about body-frame
     *                     y-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect ECI frame, resolved about body-frame
     *                     z-axis, averaged over time interval and expressed in radians per second (rad/s).
     */
    public ECIKinematics(final double fx, final double fy, final double fz,
                         final double angularRateX, final double angularRateY,
                         final double angularRateZ) {
        super(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI frame resolved along body-frame x-axis,
     *                       averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI frame resolved along body-frame y-axis,
     *                       averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI frame resolved along body-frame z-axis,
     *                       averaged over time interval.
     */
    public ECIKinematics(final Acceleration specificForceX,
                         final Acceleration specificForceY,
                         final Acceleration specificForceZ) {
        super(specificForceX, specificForceY, specificForceZ);
    }

    /**
     * Constructor.
     *
     * @param angularSpeedX Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public ECIKinematics(final AngularSpeed angularSpeedX,
                         final AngularSpeed angularSpeedY,
                         final AngularSpeed angularSpeedZ) {
        super(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI frame resolved along body-frame x-axis,
     *                       averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI frame resolved along body-frame y-axis,
     *                       averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI frame resolved along body-frame z-axis,
     *                       averaged over time interval.
     * @param angularSpeedX  Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                       x-axis, averaged over time interval.
     * @param angularSpeedY  Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                       y-axis, averaged over time interval.
     * @param angularSpeedZ  Angular speed of body frame with respect ECI frame, resolved about body-frame
     *                       z-axis, averaged over time interval.
     */
    public ECIKinematics(final Acceleration specificForceX,
                         final Acceleration specificForceY,
                         final Acceleration specificForceZ,
                         final AngularSpeed angularSpeedX,
                         final AngularSpeed angularSpeedY,
                         final AngularSpeed angularSpeedZ) {
        super(specificForceX, specificForceY, specificForceZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public ECIKinematics(final ECIKinematics input) {
        super(input);
    }

    /**
     * Checks if provided object is an ECIKinematics instance having exactly the same contents
     * as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof ECIKinematics)) {
            return false;
        }

        final ECIKinematics other = (ECIKinematics) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"CloneDoesntDeclareCloneNotSupportedException", "MethodDoesntCallSuperMethod"})
    @Override
    protected Object clone() {
        return new ECIKinematics(this);
    }
}
