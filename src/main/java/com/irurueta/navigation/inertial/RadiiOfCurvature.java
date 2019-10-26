package com.irurueta.navigation.inertial;

import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains radii of curvature of the WGS84 ellipsoid at a given latitude.
 */
@SuppressWarnings("WeakerAccess")
public class RadiiOfCurvature implements Serializable, Cloneable {

    /**
     * Meridian radius of curvature expressed in meters (m).
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     */
    private double mRn;

    /**
     * Transverse radius of curvature expressed in meters (m).
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     */
    private double mRe;

    /**
     * Constructor.
     */
    public RadiiOfCurvature() {
    }

    /**
     * Constructor.
     *
     * @param rn meridian radius of curvature expressed in meters (m).
     * @param re transverse radius of curvature expressed in meters (m).
     */
    public RadiiOfCurvature(final double rn, final double re) {
        setValues(rn, re);
    }

    /**
     * Constructor.
     *
     * @param rnDistance meridian radius of curvature.
     * @param reDistance transverse radius of curvature.
     */
    public RadiiOfCurvature(final Distance rnDistance, final Distance reDistance) {
        setValues(rnDistance, reDistance);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public RadiiOfCurvature(final RadiiOfCurvature input) {
        copyFrom(input);
    }

    /**
     * Gets meridian radius of curvature expressed in meters (m).
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     *
     * @return meridian radius of curvature expressed in meters (m).
     */
    public double getRn() {
        return mRn;
    }

    /**
     * Sets meridian radius of curvature expressed in meters (m).
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     *
     * @param rn meridian radius of curvature expressed in meters (m).
     */
    public void setRn(final double rn) {
        mRn = rn;
    }

    /**
     * Gets transverse radius of curvature expressed in meters (m).
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     *
     * @return transverse radius of curvature expressed in meters (m).
     */
    public double getRe() {
        return mRe;
    }

    /**
     * Sets transverse radius of curvature expressed in meters (m).
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     *
     * @param re transverse radius of curvature expressed in meters (m).
     */
    public void setRe(final double re) {
        mRe = re;
    }

    /**
     * Sets radii of curvature.
     *
     * @param rn meridian radius of curvature expressed in meters (m).
     * @param re transverse radius of curvature expressed in meters (m).
     */
    public void setValues(final double rn, final double re) {
        mRn = rn;
        mRe = re;
    }

    /**
     * Gets meridian radius of curvature.
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     *
     * @param result instance where meridian radius of curvature will be stored.
     */
    public void getRnDistance(final Distance result) {
        result.setValue(mRn);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets meridian radius of curvature.
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     *
     * @return meridian radius of curvature.
     */
    public Distance getRnDistance() {
        return new Distance(mRn, DistanceUnit.METER);
    }

    /**
     * Sets meridian radius of curvature.
     * This is the radius of curvature for north-south motion.
     * It is the radius of curvature of a meridian, a cross-section of the ellipsoid
     * surface in the north-down plane, at the point of interest (a given latitude).
     * This is the same as the radius of the best-fitting circle to the meridian
     * ellipse at the point of interest.
     * The meridian radius of curvature varies with latitude and is smallest at the
     * equator, where the geocentric radius is largest, and largest at the poles.
     *
     * @param rnDistance meridian radius of curvature to be set.
     */
    public void setRnDistance(final Distance rnDistance) {
        mRn = DistanceConverter.convert(rnDistance.getValue().doubleValue(),
                rnDistance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets transverse radius or curvature.
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     *
     * @param result instance where transverse radius of curvature will be stored.
     */
    public void getReDistance(final Distance result) {
        result.setValue(mRe);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets transverse radius of curvature.
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     *
     * @return transverse radius of curvature.
     */
    public Distance getReDistance() {
        return new Distance(mRe, DistanceUnit.METER);
    }

    /**
     * Sets transverse radius of curvature.
     * This is the radius of curvature for east-wet motion.
     * This is also known as the normal radius of curvature or prime vertical radius
     * of curvature.
     * It is the radius of curvature of a cross-section of the ellipsoid surface in
     * the east-down plane at the point of interest.
     * This is the vertical plane perpendicular to the meridian plane and is not
     * the plane of constant latitude.
     * The transverse radius of curvature varies with latitude and is smallest at the
     * equator. It is also equal to the length of the normal from a point on the
     * surface to the polar axis.
     *
     * @param reDistance transverse radius of curvature to be set.
     */
    public void setReDistance(final Distance reDistance) {
        mRe = DistanceConverter.convert(reDistance.getValue().doubleValue(),
                reDistance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets radii of curvature.
     *
     * @param rnDistance meridian radius of curvature.
     * @param reDistance transverse radius of curvature.
     */
    public void setValues(final Distance rnDistance, final Distance reDistance) {
        setRnDistance(rnDistance);
        setReDistance(reDistance);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final RadiiOfCurvature output) {
        output.mRn = mRn;
        output.mRe = mRe;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final RadiiOfCurvature input) {
        mRn = input.mRn;
        mRe = input.mRe;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mRn, mRe);
    }

    /**
     * Checks if provided object is a RadiiOfCurvature instance having exactly the same
     * contents as this instance.
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
        if (!(obj instanceof RadiiOfCurvature)) {
            return false;
        }

        final RadiiOfCurvature other = (RadiiOfCurvature) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final RadiiOfCurvature other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between radii values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final RadiiOfCurvature other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mRn - other.mRn) <= threshold
                && Math.abs(mRe - other.mRe) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new RadiiOfCurvature(this);
    }
}
