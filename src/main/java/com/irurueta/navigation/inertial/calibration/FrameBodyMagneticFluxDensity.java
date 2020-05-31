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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;

import java.io.Serializable;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Objects;

/**
 * Contains a body magnetic flux density along with the corresponding frame
 * (position, orientatoin and velocity) where the measurement was made.
 */
public class FrameBodyMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Current body magnetic flux density. Contains magnetometer measurements.
     */
    private BodyMagneticFluxDensity mMagneticFluxDensity;

    /**
     * Contains current body position, velocity (which will typically be zero)
     * and orientation resolved around ECEF axes.
     */
    private ECEFFrame mFrame;

    /**
     * Contains year expressed in decimal format.
     */
    private double mYear;

    /**
     * Constructor.
     */
    public FrameBodyMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     */
    public FrameBodyMagneticFluxDensity(final ECEFFrame frame) {
        mFrame = frame;
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     */
    public FrameBodyMagneticFluxDensity(final NEDFrame frame) {
        setNedFrame(frame);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame) {
        this(magneticFluxDensity);
        mFrame = frame;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current NED frame associated to measurement.
     *                            Internally it will be converted to its
     *                            corresponding ECEF frame.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame) {
        this(magneticFluxDensity);
        setNedFrame(frame);
    }

    /**
     * Constructor.
     *
     * @param year time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(final double year) {
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param year                time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double year) {
        mMagneticFluxDensity = magneticFluxDensity;
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     * @param year  time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final double year) {
        mFrame = frame;
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     * @param year  time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(
            final NEDFrame frame, final double year) {
        setNedFrame(frame);
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param year                time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final double year) {
        this(magneticFluxDensity);
        mFrame = frame;
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current NED frame associated to measurement.
     *                            Internally it will be converted to its
     *                            corresponding ECEF frame.
     * @param year                time expressed as decimal year.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final double year) {
        this(magneticFluxDensity);
        setNedFrame(frame);
        mYear = year;
    }

    /**
     * Constructor.
     *
     * @param time a timestamp.
     */
    public FrameBodyMagneticFluxDensity(final Date time) {
        this(convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param time                a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final Date time) {
        this(magneticFluxDensity, convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     * @param time  a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final Date time) {
        this(frame, convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     * @param time  a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final NEDFrame frame, final Date time) {
        this(frame, convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param time                a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final Date time) {
        this(magneticFluxDensity, frame, convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current NED frame associated to measurement.
     *                            Internally it will be converted to its
     *                            corresponding ECEF frame.
     * @param time                a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final Date time) {
        this(magneticFluxDensity, frame, convertTime(time));
    }

    /**
     * Constructor.
     *
     * @param calendar calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(final GregorianCalendar calendar) {
        this(convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param calendar            calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final GregorianCalendar calendar) {
        this(magneticFluxDensity, convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param frame    current ECEF frame associated to measurement.
     * @param calendar calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final GregorianCalendar calendar) {
        this(frame, convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param frame    current NED frame associated to measurement. Internally it
     *                 will be converted to its corresponding ECEF frame.
     * @param calendar calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final NEDFrame frame, final GregorianCalendar calendar) {
        this(frame, convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param calendar            calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final GregorianCalendar calendar) {
        this(magneticFluxDensity, frame, convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current NED frame associated to measurement.
     *                            Internally it will be converted to its
     *                            corresponding ECEF frame.
     * @param calendar            calendar containing a timestamp.
     */
    public FrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final GregorianCalendar calendar) {
        this(magneticFluxDensity, frame, convertTime(calendar));
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public FrameBodyMagneticFluxDensity(
            final FrameBodyMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets current body magnetic flux density. Contains magnetometer
     * measurements.
     *
     * @return current body magnetic flux density.
     */
    public BodyMagneticFluxDensity getMagneticFluxDensity() {
        return mMagneticFluxDensity;
    }

    /**
     * Sets current body magnetic flux density. Contains magnetometer
     * measurements.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     */
    public void setMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity(which will typically be zero) and orientation (which usually
     * changes with each measurement to perform calibration of a single device)
     * resolved around ECEF axes associated to body magnetic flux density
     * measurement.
     *
     * @return current ECEF frame associated to body magnetic flux density
     * measurement or null if not available.
     */
    public ECEFFrame getFrame() {
        return mFrame;
    }

    /**
     * Sets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually
     * changes with each measurement to perform calibration of a single
     * device) resolved around ECEF axes associated to body magnetic flux
     * density measurement.
     *
     * @param frame current ECEF frame.
     */
    public void setFrame(final ECEFFrame frame) {
        mFrame = frame;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually
     * changes with each measurement to perform calibration of a single
     * device) resolved around NED axes associated to body magnetic flux
     * density measurement.
     *
     * @return current NED frame associated to body magnetic flux density
     * measurement or null if not available.
     */
    public NEDFrame getNedFrame() {
        return mFrame != null ?
                ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(mFrame) : null;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually
     * changes with each measurement to perform calibration of a single
     * device) resolved around NED axes associated to body magnetic flux
     * density measurement.
     *
     * @param result instance where result data will be stored if available.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getNedFrame(final NEDFrame result) {
        if (mFrame != null) {
            ECEFtoNEDFrameConverter.convertECEFtoNED(mFrame, result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually
     * changes with each measurement to perform calibration of a single
     * device) resolved around NED axes associated to body magnetic flux
     * density measurement.
     * <p>
     * This method will internally store the corresponding ECEF frame to provided
     * NED frame value.
     *
     * @param nedFrame current NED frame associated to body magnetic flux
     *                 density measurement to be set.
     */
    public void setNedFrame(final NEDFrame nedFrame) {
        if (nedFrame != null) {
            if (mFrame != null) {
                NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
            } else {
                mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            }
        } else {
            mFrame = null;
        }
    }

    /**
     * Gets year expressed in decimal format.
     *
     * @return year expressed in decimal format.
     */
    public double getYear() {
        return mYear;
    }

    /**
     * Sets year expressed in decimal format.
     *
     * @param year year expressed in decimal format.
     */
    public void setYear(final double year) {
        mYear = year;
    }

    /**
     * Sets decimal year from provided date instance.
     *
     * @param date a date instance containing a timestamp.
     */
    public void setTime(final Date date) {
        mYear = convertTime(date);
    }

    /**
     * Sets decimal year from provided calendar instance.
     *
     * @param calendar a calendar instance containing a timestamp.
     */
    public void setTime(final GregorianCalendar calendar) {
        mYear = convertTime(calendar);
    }

    /**
     * Converts a time instant contained ina date object to a
     * decimal year.
     *
     * @param date a time instance to be converted.
     * @return converted value expressed in decimal years.
     */
    public static double convertTime(final Date date) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(date);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained in a gregorian calendar to a
     * decimal year.
     *
     * @param calendar calendar containing a specific instant to be
     *                 converted.
     * @return converted value expressed in decimal years.
     */
    public static double convertTime(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy dara from.
     */
    public void copyFrom(final FrameBodyMagneticFluxDensity input) {
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

        if (input.mFrame != null) {
            if (mFrame == null) {
                mFrame = new ECEFFrame(input.mFrame);
            } else {
                mFrame.copyFrom(input.mFrame);
            }
        } else {
            mFrame = null;
        }

        mYear = input.mYear;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final FrameBodyMagneticFluxDensity output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mMagneticFluxDensity, mFrame, mYear);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final FrameBodyMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between magnetic flux density and
     *                  frame values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(
            final FrameBodyMagneticFluxDensity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.mMagneticFluxDensity == null && mMagneticFluxDensity == null)
                || (mMagneticFluxDensity != null && mMagneticFluxDensity.equals(other.mMagneticFluxDensity, threshold)))
                && ((other.mFrame == null && mFrame == null)
                || (mFrame != null && mFrame.equals(other.mFrame, threshold)))
                && Math.abs(other.mYear - mYear) <= threshold;
    }

    /**
     * Checks if provided object is a FrameBodyMagneticFluxDensity instance
     * having exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final FrameBodyMagneticFluxDensity other =
                (FrameBodyMagneticFluxDensity) obj;
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
        final FrameBodyMagneticFluxDensity result =
                (FrameBodyMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
