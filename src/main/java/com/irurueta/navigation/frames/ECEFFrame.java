package com.irurueta.navigation.frames;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains position, velocity and coordinates transformation matrix expressed in ECEF frame.
 * Position and velocity of this frame is expressed along ECEF axes as described here:
 * {@link FrameType#EARTH_CENTERED_EARTH_FIXED_FRAME}.
 */
@SuppressWarnings("WeakerAccess")
public class ECEFFrame implements Frame, Serializable, Cloneable {

    /**
     * Number of coordinates representing position.
     */
    public static final int NUM_POSITION_COORDINATES = 3;

    /**
     * Number of coordinates representing velocity.
     */
    public static final int NUM_VELOCITY_COORDINATES = 3;

    /**
     * Cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     * ECEF-frame axes.
     */
    private double mX;

    /**
     * Catesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     * ECEF-frame axes.
     */
    private double mY;

    /**
     * Cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     * ECEF-frame axes.
     */
    private double mZ;

    /**
     * X coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame,
     * resolved along ECEF-frame axes.
     */
    private double mVx;

    /**
     * Y coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame,
     * resolved along ECEF-frame axes.
     */
    private double mVy;

    /**
     * Z coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame,
     * resolved along ECEF-frame axes.
     */
    private double mVz;

    /**
     * Body to ECEF frame coordinate transformation matrix.
     */
    private CoordinateTransformationMatrix mC;

    /**
     * Constructor.
     * Initializes position and velocity coordinates to zero and the coordinate transformation matrix to the
     * identity.
     */
    public ECEFFrame() {
        mC = new CoordinateTransformationMatrix(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
    }

    /**
     * Constructor.
     *
     * @param c Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final CoordinateTransformationMatrix c) throws InvalidSourceAndDestinationFrameTypeException {
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z) {
        this();
        setCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position) {
        this();
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ) {
        this();
        setPositionCoordinates(positionX, positionY, positionZ);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param y  cartesian y coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param z  cartesian z coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param vx x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vy y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vz z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final double vx, final double vy, final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position body position expresed in meters (m) and resolved along ECEF-frame axes.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position,
                     final double vx, final double vy, final double vz) {
        this(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param speedX   x coordinate of velocity to be set.
     * @param speedY   y coordinate of velocity to be set.
     * @param speedZ   z coordinate of velocity to be set.
     */
    public ECEFFrame(final Point3D position,
                     final Speed speedX, final Speed speedY, final Speed speedZ) {
        this(position);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param speedX x coordinate of velocity to be set.
     * @param speedY y coordinate of velocity to be set.
     * @param speedZ z coordinate of velocity to be set.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final Speed speedX, final Speed speedY, final Speed speedZ) {
        this(x, y, z);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     * @param vx        x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vy        y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vz        z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final double vx,
                     final double vy, final double vz) {
        this(positionX, positionY, positionZ);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     * @param speedX    x coordinate of velocity to be set.
     * @param speedY    y coordinate of velocity to be set.
     * @param speedZ    z coordinate of velocity to be set.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final Speed speedX,
                     final Speed speedY, final Speed speedZ) {
        this(positionX, positionY, positionZ);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param c Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z, final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {

        this(position);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {

        this(positionX, positionY, positionZ);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param y  cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param z  cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param vx x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vy y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vz z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param c  Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final double vx, final double vy, final double vz,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z, vx, vy, vz);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expresed in meters (m) and resolved along ECEF-frame axes.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final double vx, final double vy, final double vz,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, vx, vy, vz);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param speedX   x coordinate of velocity to be set.
     * @param speedY   y coordinate of velocity to be set.
     * @param speedZ   z coordinate of velocity to be set.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final Speed speedX, final Speed speedY, final Speed speedZ,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, speedX, speedY, speedZ);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *               ECEF-frame axes.
     * @param speedX x coordinate of velocity to be set.
     * @param speedY y coordinate of velocity to be set.
     * @param speedZ z coordinate of velocity to be set.
     * @param c      Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final Speed speedX, final Speed speedY, final Speed speedZ,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z, speedX, speedY, speedZ);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     * @param vx        x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vy        y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vz        z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final double vx,
                     final double vy, final double vz,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(positionX, positionY, positionZ, vx, vy, vz);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     * @param speedX    x coordinate of velocity to be set.
     * @param speedY    y coordinate of velocity to be set.
     * @param speedZ    z coordinate of velocity to be set.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final Speed speedX,
                     final Speed speedY, final Speed speedZ,
                     final CoordinateTransformationMatrix c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(positionX, positionY, positionZ, speedX, speedY, speedZ);
        setCoordinateTransformationMatrix(c);
    }

    /**
     * Constructor.
     *
     * @param input ECEF frame to copy data from.
     */
    public ECEFFrame(final ECEFFrame input) {
        this();
        copyFrom(input);
    }

    /**
     * Gets cartesian x coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return cartesian x coordinate of body position.
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets cartesian x coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position.
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets cartesian y coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return cartesian y coordinate of body position.
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets cartesian y coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param y cartesian y coordinate of body position.
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets cartesian z coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return cartesian z coordinate of body position.
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets cartesian z coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param z cartesian z coordinate of body position.
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets cartesian coordinates of body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param x cartesian x coordinate of body position.
     * @param y cartesian y coordinate of body position.
     * @param z cartesian z coordinate of body position.
     */
    public void setCoordinates(final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Get body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @return body position.
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Gets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param result instance where position data is copied to.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Sets body position expressed in meters (m) and resolved along ECEF-frame axes.
     *
     * @param point body position to be set.
     */
    public void setPosition(final Point3D point) {
        mX = point.getInhomX();
        mY = point.getInhomY();
        mZ = point.getInhomZ();
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian x coordinate of body position will be stored.
     */
    public void getPositionX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @return x coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getPositionX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets cartesian x coordinate of body position resolved along ECEF-frame axes.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     */
    public void setPositionX(final Distance positionX) {
        mX = DistanceConverter.convert(positionX.getValue().doubleValue(),
                positionX.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian x coordinate of body position will be stored.
     */
    public void getPositionY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @return y coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getPositionY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets cartesian y coordinate of body position resolved along ECEF-frame axes.
     *
     * @param positionY cartesian y coordinate of body position to be set.
     */
    public void setPositionY(final Distance positionY) {
        mY = DistanceConverter.convert(positionY.getValue().doubleValue(),
                positionY.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param result instance where cartesian z coordinate of body position will be stored.
     */
    public void getPositionZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @return z coordinate of body position resolved along ECEF-frame axes.
     */
    public Distance getPositionZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets cartesian z coordinate of body position resolved along ECEF-frame axes.
     *
     * @param positionZ cartesian z coordinate of body position to be set.
     */
    public void setPositionZ(final Distance positionZ) {
        mZ = DistanceConverter.convert(positionZ.getValue().doubleValue(),
                positionZ.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets cartesian coordinates of body position resolved along ECEF-frame axes.
     *
     * @param positionX cartesian x coordinate of body position to be set.
     * @param positionY cartesian y coordinate of body position to be set.
     * @param positionZ cartesian z coordinate of body position to be set.
     */
    public void setPositionCoordinates(final Distance positionX,
                                       final Distance positionY,
                                       final Distance positionZ) {
        setPositionX(positionX);
        setPositionY(positionY);
        setPositionZ(positionZ);
    }

    /**
     * Gets norm of position expressed in meters (m), which represents the distance to
     * Earth's center of mass.
     *
     * @return position norm expressed in meters (m).
     */
    public double getPositionNorm() {
        return Math.sqrt(mX * mX + mY * mY + mZ * mZ);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionNormAsDistance(final Distance result) {
        result.setValue(getPositionNorm());
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets norm of position, which represents the distance to Earth's center of mass.
     *
     * @return position norm.
     */
    public Distance getPositionNormAsDistance() {
        return new Distance(getPositionNorm(), DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @return x coordinate of velocity.
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets x coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets y coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity.
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets y coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @param vy y coordinate of velocity.
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets z coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity.
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets z coordinate of velocity of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @param vz z coordinate of velocity.
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s) resolved along ECEF-frame axes.
     *
     * @param vx x coordinate of velocity.
     * @param vy y coordinate of velocity.
     * @param vz z coordinate of velocity.
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getVelocityNorm() {
        return Math.sqrt(mVx * mVx + mVy * mVy + mVz * mVz);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @param result velocity norm.
     */
    public void getVelocityNormAsSpeed(final Speed result) {
        result.setValue(getVelocityNorm());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @return velocity norm.
     */
    public Speed getVelocityNormAsSpeed() {
        return new Speed(getVelocityNorm(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param result instance where x coordinate of velocity will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return x coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets x coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param speedX x coordinate of velocity of body frame resolved along ECEF-frame
     *               axes to be set.
     */
    public void setSpeedX(final Speed speedX) {
        mVx = SpeedConverter.convert(speedX.getValue().doubleValue(),
                speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param result instance where y coordinate of velocity will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return y coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets y coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param speedY y coordinate of velocity of body frame resolved along ECEF-frame
     *               axes to be set.
     */
    public void setSpeedY(final Speed speedY) {
        mVy = SpeedConverter.convert(speedY.getValue().doubleValue(),
                speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param result instance where z coordinate of velocity will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @return z coordinate of velocity of body frame resolved along ECEF-frame axes.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets z coordinate of velocity of body frame resolved along ECEF-frame axes.
     *
     * @param speedZ z coordinate of velocity of body frame resolved along ECEF-frame
     *               axes to be set.
     */
    public void setSpeedZ(final Speed speedZ) {
        mVz = SpeedConverter.convert(speedZ.getValue().doubleValue(),
                speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity coordinates of body frame resolved along ECEF-frame axes.
     *
     * @param speedX x coordinate of velocity to be set.
     * @param speedY y coordinate of velocity to be set.
     * @param speedZ z coordinate of velocity to be set.
     */
    public void setSpeedCoordinates(final Speed speedX,
                                    final Speed speedY,
                                    final Speed speedZ) {
        setSpeedX(speedX);
        setSpeedY(speedY);
        setSpeedZ(speedZ);
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public CoordinateTransformationMatrix getCoordinateTransformationMatrix() {
        CoordinateTransformationMatrix result = new CoordinateTransformationMatrix(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
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
     * Provided value must be a body to ECEF transformation matrix.
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
     * Only body to ECEF transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidCoordinateTransformationMatrix(final CoordinateTransformationMatrix c) {
        return c.getSourceType() == FrameType.BODY_FRAME &&
                c.getDestinationType() == FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final ECEFFrame output) {
        output.mX = mX;
        output.mY = mY;
        output.mZ = mZ;

        output.mVx = mVx;
        output.mVy = mVy;
        output.mVz = mVz;

        mC.copyTo(output.mC);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final ECEFFrame input) {
        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;

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
        return Objects.hash(mX, mY, mZ, mVx, mVy, mVz, mC);
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
        if (!(obj instanceof ECEFFrame)) {
            return false;
        }

        final ECEFFrame other = (ECEFFrame) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final ECEFFrame other) {
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
    public boolean equals(final ECEFFrame other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold
                && mC.equals(other.mC, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new ECEFFrame(this);
    }
}
