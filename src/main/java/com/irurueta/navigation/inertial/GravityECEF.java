package com.irurueta.navigation.inertial;

import com.irurueta.units.Acceleration;

/**
 * Contains acceleration due to gravity resolved about ECEF frame.
 */
public class GravityECEF extends GravityOrGravitation<GravityECEF> {


    /**
     * Constructor.
     */
    public GravityECEF() {
        super();
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public GravityECEF(final double gx, final double gy, final double gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis to be set.
     * @param gy acceleration due to gravity through ECEF y-axis to be set.
     * @param gz acceleration due to gravity through ECEF z-axis to be set.
     */
    public GravityECEF(final Acceleration gx, final Acceleration gy,
                       final Acceleration gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public GravityECEF(final GravityECEF input) {
        super(input);
    }

    /**
     * Checks if provided object is a GravityECEF instance having exactly the same contents
     * as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof GravityECEF)) {
            return false;
        }

        final GravityECEF other = (GravityECEF) obj;
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
        return new GravityECEF(this);
    }
}
