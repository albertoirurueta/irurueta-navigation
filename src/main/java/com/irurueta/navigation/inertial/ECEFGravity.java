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

/**
 * Contains acceleration due to gravity resolved about ECEF frame.
 */
public class ECEFGravity extends GravityOrGravitation<ECEFGravity> {


    /**
     * Constructor.
     */
    public ECEFGravity() {
        super();
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public ECEFGravity(final double gx, final double gy, final double gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECEF x-axis to be set.
     * @param gy acceleration due to gravity through ECEF y-axis to be set.
     * @param gz acceleration due to gravity through ECEF z-axis to be set.
     */
    public ECEFGravity(final Acceleration gx, final Acceleration gy,
                       final Acceleration gz) {
        super(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public ECEFGravity(final ECEFGravity input) {
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
        if (!(obj instanceof ECEFGravity)) {
            return false;
        }

        final ECEFGravity other = (ECEFGravity) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final ECEFGravity result = (ECEFGravity) super.clone();
        copyTo(result);
        return result;
    }
}
