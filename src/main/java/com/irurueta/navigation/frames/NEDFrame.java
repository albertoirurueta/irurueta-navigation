package com.irurueta.navigation.frames;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains position, velocity and coordinates transformation matrix expressed in NED frame.
 * Position is expressed as latitude, longitude and height.
 * Velocity of body frame is expressed with respect ECEF frame and resolved along north, east and down axes,
 * as defined in {@link FrameType#LOCAL_NAVIGATION_FRAME}.
 */
@SuppressWarnings("WeakerAccess")
public class NEDFrame implements Frame, Serializable, Cloneable {

    /**
     * Number of coordinates representing velocity.
     */
    public static final int NUM_VELOCITY_COORDINATES = 3;

    /**
     * Latitude expressed in radians.
     */
    private double mLatitude;

    /**
     * Longitude expressed in radians.
     */
    private double mLongitude;

    /**
     * Height expressed in meters.
     */
    private double mHeight;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     */
    private double mVn;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     */
    private double mVe;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     */
    private double mVd;

    /**
     * Body to NED coordinate transformation matrix.
     */
    private CoordinateTransformationMatrix mC;

    /**
     * Constructor.
     * Initializes position and velocity coordinates to zero and the coordinate transformation matrix to the
     * identity.
     */
    public NEDFrame() {
        mC = new CoordinateTransformationMatrix(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
    }

    /**
     * Constructor.
     *
     * @param c Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final CoordinateTransformationMatrix c) throws InvalidSourceAndDestinationFrameTypeException {
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     */
    public NEDFrame(final double latitude, final double longitude, final double height) {
        this();
        setPosition(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final double vn, final double ve, final double vd) {
        this(latitude, longitude, height);
        setVelocityCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final CoordinateTransformationMatrix c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final double vn, final double ve, final double vd,
                    final CoordinateTransformationMatrix c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, vn, ve, vd);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param input NED frame to copy data from.
     */
    public NEDFrame(final NEDFrame input) {
        this();
        copyFrom(input);
    }

    /**
     * Gets latitude expressed in radians.
     *
     * @return latitude expressed in radians.
     */
    public double getLatitude() {
        return mLatitude;
    }

    /**
     * Sets latitude expressed in radians.
     *
     * @param latitude latitude expressed in radians.
     */
    public void setLatitude(final double latitude) {
        mLatitude = latitude;
    }

    /**
     * Gets longitude expressed in radians.
     *
     * @return longitude expressed in radians.
     */
    public double getLongitude() {
        return mLongitude;
    }

    /**
     * Sets longitude expressed in radians.
     *
     * @param longitude longitude expressed in radians.
     */
    public void setLongitude(final double longitude) {
        mLongitude = longitude;
    }

    /**
     * Gets height expressed in meters.
     *
     * @return height expressed in meters.
     */
    public double getHeight() {
        return mHeight;
    }

    /**
     * Sets height expressed in meters.
     *
     * @param height height expressed in meters.
     */
    public void setHeight(double height) {
        mHeight = height;
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     */
    public void setPosition(final double latitude, final double longitude, final double height) {
        mLatitude = latitude;
        mLongitude = longitude;
        mHeight = height;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate value.
     */
    public double getVn() {
        return mVn;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @param vn North velocity coordinate value.
     */
    public void setVn(final double vn) {
        mVn = vn;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate value.
     */
    public double getVe() {
        return mVe;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @param ve East velocity coordinate value.
     */
    public void setVe(final double ve) {
        mVe = ve;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate value.
     */
    public double getVd() {
        return mVd;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param vd Down velocity coordinate value.
     */
    public void setVd(final double vd) {
        mVd = vd;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s) resolved along North, East, Down
     * axes.
     *
     * @param vn North velocity coordinate value.
     * @param ve East velocity coordinate value.
     * @param vd Down velocity coordinate value.
     */
    public void setVelocityCoordinates(final double vn, final double ve, final double vd) {
        mVn = vn;
        mVe = ve;
        mVd = vd;
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public CoordinateTransformationMatrix getCoordinateTransformationMatrix() {
        CoordinateTransformationMatrix result = new CoordinateTransformationMatrix(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        getCoordinateTransformationMatrix(result);
        return result;
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    @Override
    public void getCoordinateTransformationMatrix(final CoordinateTransformationMatrix result) {
        mC.copyTo(result);
    }

    /**
     * Sets coordinate transformation matrix.
     * Provided value must be a body to NED transformation matrix.
     *
     * @param c coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    @Override
    public void setCoordinateTransformationMatrix(final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        if (!isValidCoordinateTransformationMatrix(c)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        mC = c;
    }

    /**
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to NED transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidCoordinateTransformationMatrix(final CoordinateTransformationMatrix c) {
        return c.getSourceType() == FrameType.BODY_FRAME &&
                c.getDestinationType() == FrameType.LOCAL_NAVIGATION_FRAME;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDFrame output) {
        output.mLatitude = mLatitude;
        output.mLongitude = mLongitude;
        output.mHeight = mHeight;

        output.mVn = mVn;
        output.mVe = mVe;
        output.mVd = mVd;

        mC.copyTo(output.mC);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDFrame input) {
        mLatitude = input.mLatitude;
        mLongitude = input.mLongitude;
        mHeight = input.mHeight;

        mVn = input.mVn;
        mVe = input.mVe;
        mVd = input.mVd;

        mC.copyFrom(input.mC);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mLatitude, mLongitude, mHeight, mVn, mVe, mVd, mC);
    }

    /**
     * Checks if provided object is an ECEFFrame having exactly the same contents as
     * this instance.
     *
     * @param obj Object to be compared.
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
        if (!(obj instanceof NEDFrame)) {
            return false;
        }

        NEDFrame other = (NEDFrame) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final NEDFrame other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position, velocity and coordinate transformation matrix.
     * @return true if both instances are considered to be equal (up to provided threshold), false otherwise.
     */
    public boolean equals(final NEDFrame other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mLatitude - other.mLatitude) <= threshold &&
                Math.abs(mLongitude - other.mLongitude) <= threshold &&
                Math.abs(mHeight - other.mHeight) <= threshold &&
                Math.abs(mVn - other.mVn) <= threshold &&
                Math.abs(mVe - other.mVe) <= threshold &&
                Math.abs(mVd - other.mVd) <= threshold &&
                mC.equals(other.mC, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new NEDFrame(this);
    }
}
