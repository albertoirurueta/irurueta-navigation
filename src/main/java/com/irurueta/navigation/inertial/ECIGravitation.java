package com.irurueta.navigation.inertial;

import com.irurueta.units.Acceleration;

/**
 * Contains acceleration due to gravity resolved about ECI frame.
 */
@SuppressWarnings("WeakerAccess")
public class ECIGravitation extends GravityOrGravitation<ECIGravitation> {

    /**
     * Constructor.
     */
    public ECIGravitation() {
        super();
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECI x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECI y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECI z-axis expressed in meters per squared second (m/s^2).
     */
    public ECIGravitation(final double gx, final double gy, final double gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECI x-axis to be set.
     * @param gy acceleration due to gravity through ECI y-axis to be set.
     * @param gz acceleration due to gravity through ECI z-axis to be set.
     */
    public ECIGravitation(final Acceleration gx, final Acceleration gy,
                          final Acceleration gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public ECIGravitation(final ECIGravitation input) {
        super(input);
    }

    /**
     * Check if provided object is a GravitationECI instance having exactly the same contents
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
        if (!(obj instanceof ECIGravitation)) {
            return false;
        }

        final ECIGravitation other = (ECIGravitation) obj;
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
        return new ECIGravitation(this);
    }
}
