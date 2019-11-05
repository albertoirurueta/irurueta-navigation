/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial;

import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;

/**
 * Describes the motion of a body based on the specific forces (i.e. specific acceleration) and
 * angular rates applied to it with respect and resolved along the NED frame.
 * Body frame axes are typically defined so that x is the forward axis, pointing in the usual direction
 * of travel, z is the down axis, pointing in the usual direction of gravity, and y is the right axis,
 * completing the orthogonal set.
 */
@SuppressWarnings("WeakerAccess")
public class NEDKinematics extends FrameKinematics<NEDKinematics> {

    /**
     * Constructor.
     */
    public NEDKinematics() {
        super();
    }

    /**
     * Constructor.
     *
     * @param fx Specific force of body frame with respect NED frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect NED frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect NED frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public NEDKinematics(final double fx, final double fy, final double fz) {
        super(fx, fy, fz);
    }

    /**
     * Constructor.
     *
     * @param fx           Specific force of body frame with respect NED frame resolved along body-frame x-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy           Specific force of body frame with respect NED frame resolved along body-frame y-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz           Specific force of body frame with respect NED frame resolved along body-frame z-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param angularRateX Angular rate of body frame with respect NED frame, resolved about body-frame
     *                     x-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateY Angular rate of body frame with respect NED frame, resolved about body-frame
     *                     y-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect NED frame, resolved about body-frame
     *                     z-axis, averaged over time interval and expressed in radians per second (rad/s).
     */
    public NEDKinematics(final double fx, final double fy, final double fz,
                         final double angularRateX, final double angularRateY,
                         final double angularRateZ) {
        super(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect NED frame resolved along body-frame x-axis,
     *                       averaged over time interval.
     * @param specificForceY Specific force of body frame with respect NED frame resolved along body-frame y-axis,
     *                       averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect NED frame resolved along body-frame z-axis,
     *                       averaged over time interval.
     */
    public NEDKinematics(final Acceleration specificForceX,
                         final Acceleration specificForceY,
                         final Acceleration specificForceZ) {
        super(specificForceX, specificForceY, specificForceZ);
    }

    /**
     * Constructor.
     *
     * @param angularSpeedX Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public NEDKinematics(final AngularSpeed angularSpeedX,
                         final AngularSpeed angularSpeedY,
                         final AngularSpeed angularSpeedZ) {
        super(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect NED frame resolved along body-frame x-axis,
     *                       averaged over time interval.
     * @param specificForceY Specific force of body frame with respect NED frame resolved along body-frame y-axis,
     *                       averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect NED frame resolved along body-frame z-axis,
     *                       averaged over time interval.
     * @param angularSpeedX Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect NED frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public NEDKinematics(final Acceleration specificForceX,
                         final Acceleration specificForceY,
                         final Acceleration specificForceZ,
                         final AngularSpeed angularSpeedX,
                         final AngularSpeed angularSpeedY,
                         final AngularSpeed angularSpeedZ) {
        super(specificForceX, specificForceY, specificForceZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public NEDKinematics(final NEDKinematics input) {
        super(input);
    }

    /**
     * Checks if provided object is a NEDKinematics instance having exactly the same contents
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
        if (!(obj instanceof NEDKinematics)) {
            return false;
        }

        final NEDKinematics other = (NEDKinematics) obj;
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
        return new NEDKinematics(this);
    }
}
