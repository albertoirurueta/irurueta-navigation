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
package com.irurueta.navigation.inertial;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body kinematics describing the forces and angular rate applied to a body,
 * along with the sensed magnetic flux density resolved around body coordinates.
 */
public class BodyKinematicsAndMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Body kinematics containing sensed specific force and angular rate.
     */
    private BodyKinematics mKinematics;

    /**
     * Body magnetic flux density.
     */
    private BodyMagneticFluxDensity mMagneticFluxDensity;

    /**
     * Constructor.
     */
    public BodyKinematicsAndMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param kinematics body kinematics containing sensed specific force
     *                   and angular rate.
     */
    public BodyKinematicsAndMagneticFluxDensity(final BodyKinematics kinematics) {
        mKinematics = kinematics;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity body magnetic flux density.
     */
    public BodyKinematicsAndMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param kinematics          body kinematics containing sensed specific force
     *                            and angular rate.
     * @param magneticFluxDensity body magnetic flux density.
     */
    public BodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mKinematics = kinematics;
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyKinematicsAndMagneticFluxDensity(
            final BodyKinematicsAndMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets body kinematics containing sensed specific force and angular rate.
     *
     * @return body kinematics containing sensed specific force and angular rate.
     */
    public BodyKinematics getKinematics() {
        return mKinematics;
    }

    /**
     * Sets body kinematics containing sensed specific force and angular rate.
     *
     * @param kinematics body kinematics containing sensed specific force and
     *                   angular rate.
     */
    public void setKinematics(final BodyKinematics kinematics) {
        mKinematics = kinematics;
    }

    /**
     * Gets body magnetic flux density.
     *
     * @return body magnetic flux density.
     */
    public BodyMagneticFluxDensity getMagneticFluxDensity() {
        return mMagneticFluxDensity;
    }

    /**
     * Sets body magnetic flux density.
     *
     * @param magneticFluxDensity body magnetic flux density.
     */
    public void setMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyKinematicsAndMagneticFluxDensity input) {
        if (input.mKinematics != null) {
            if (mKinematics == null) {
                mKinematics = new BodyKinematics(input.mKinematics);
            } else {
                mKinematics.copyFrom(input.mKinematics);
            }
        } else {
            mKinematics = null;
        }

        if (input.mMagneticFluxDensity != null) {
            if (mMagneticFluxDensity == null) {
                mMagneticFluxDensity = new BodyMagneticFluxDensity(
                        input.mMagneticFluxDensity);
            } else {
                mMagneticFluxDensity.copyFrom(input.mMagneticFluxDensity);
            }
        } else {
            mMagneticFluxDensity = null;
        }
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyKinematicsAndMagneticFluxDensity output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fas classification and storage of objects in
     * collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mKinematics, mMagneticFluxDensity);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final BodyKinematicsAndMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other instance to be compared.
     * @param threshold maximum allowed difference between contents.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final BodyKinematicsAndMagneticFluxDensity other,
                          final double threshold) {
        if (other == null) {
            return false;
        }

        return  ((other.mKinematics == null && mKinematics == null)
                || (mKinematics != null && mKinematics.equals(other.mKinematics, threshold)))
                && ((other.mMagneticFluxDensity == null && mMagneticFluxDensity == null)
                || (mMagneticFluxDensity != null && mMagneticFluxDensity.equals(other.mMagneticFluxDensity, threshold)));
    }

    /**
     * Checks if provided object is a BodyKinematicsAndMagneticFluxDensity instance
     * having exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }

        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        final BodyKinematicsAndMagneticFluxDensity other = (BodyKinematicsAndMagneticFluxDensity) obj;
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
        final BodyKinematicsAndMagneticFluxDensity result = (BodyKinematicsAndMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
