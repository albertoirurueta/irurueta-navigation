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
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;

import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Objects;

/**
 * Extension of FrameBodyMagneticFluxDensity containing standard deviations of
 * measured magnetic flux densities besides their corresponding frame (position
 * and orientation) and timestamp where measurement was made.
 */
public class StandardDeviationFrameBodyMagneticFluxDensity extends
        FrameBodyMagneticFluxDensity {

    /**
     * Standard deviation of measured magnetic flux density expressed in Teslas
     * (T).
     */
    private double mMagneticFluxDensityStandardDeviation;

    /**
     * Constructor.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        super(magneticFluxDensity);
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame) {
        super(frame);
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame) {
        super(frame);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame) {
        super(magneticFluxDensity, frame);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current NED frame associated to measurement.
     *                            Internally it will be converted to its
     *                            corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame) {
        super(magneticFluxDensity, frame);
    }

    /**
     * Constructor.
     *
     * @param year time expressed as decimal year.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final double year) {
        super(year);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param year                time expressed as decimal year.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double year) {
        super(magneticFluxDensity, year);
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     * @param year  time expressed as decimal year.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final double year) {
        super(frame, year);
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     * @param year  time expressed as decimal year.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final double year) {
        super(frame, year);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param year                time expressed as decimal year.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final double year) {
        super(magneticFluxDensity, frame, year);
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
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final double year) {
        super(magneticFluxDensity, frame, year);
    }

    /**
     * Constructor.
     *
     * @param time a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final Date time) {
        super(time);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param time                a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final Date time) {
        super(magneticFluxDensity, time);
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     * @param time  a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final Date time) {
        super(frame, time);
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it
     *              will be converted to its corresponding ECEF frame.
     * @param time  a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final Date time) {
        super(frame, time);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param time                a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final Date time) {
        super(magneticFluxDensity, frame, time);
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
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final Date time) {
        super(magneticFluxDensity, frame, time);
    }

    /**
     * Constructor.
     *
     * @param calendar calendar containing a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final GregorianCalendar calendar) {
        super(calendar);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param calendar            calendar containing a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final GregorianCalendar calendar) {
        super(magneticFluxDensity, calendar);
    }

    /**
     * Constructor.
     *
     * @param frame    current ECEF frame associated to measurement.
     * @param calendar calendar containing a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final GregorianCalendar calendar) {
        super(frame, calendar);
    }

    /**
     * Constructor.
     *
     * @param frame    current NED frame associated to measurement. Internally it
     *                 will be converted to its corresponding ECEF frame.
     * @param calendar calendar containing a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final GregorianCalendar calendar) {
        super(frame, calendar);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     * @param frame               current ECEF frame associated to measurement.
     * @param calendar            calendar containing a timestamp.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final GregorianCalendar calendar) {
        super(magneticFluxDensity, frame, calendar);
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
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final GregorianCalendar calendar) {
        super(magneticFluxDensity, frame, calendar);
    }

    /**
     * Constructor.
     *
     * @param year                                 time expressed as decimal
     *                                             year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param year                                 time expressed as decimal
     *                                             year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param year                                 time expressed as decimal
     *                                             year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current NED frame
     *                                             associated to measurement.
     *                                             Internally it will be
     *                                             converted to its
     *                                             corresponding ECEF frame.
     * @param year                                 time expressed as decimal
     *                                             year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param year                                 time expressed as decimal
     *                                             year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current NED frame
     *                                             associated to measurement.
     *                                             Internally it will be
     *                                             converted to its
     *                                             corresponding ECEF frame.
     * @param year                                 time expressed as decimal year.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final double year,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, year);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux density.
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current NED frame associated
     *                                             to measurement. Internally
     *                                             it will be converted to its
     *                                             corresponding ECEF frame.
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current NED frame
     *                                             associated to measurement.
     *                                             Internally it will be
     *                                             converted to its
     *                                             corresponding ECEF frame.
     * @param time                                 a timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final Date time,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, time);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final ECEFFrame frame,
            final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                                current NED frame
     *                                             associated to measurement.
     *                                             Internally it will be
     *                                             converted to its
     *                                             corresponding ECEF frame.
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final NEDFrame frame, final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(frame, calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current ECEF frame
     *                                             associated to measurement.
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final ECEFFrame frame,
            final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param frame                                current NED frame
     *                                             associated to measurement.
     *                                             Internally it will be
     *                                             converted to its
     *                                             corresponding ECEF frame.
     * @param calendar                             calendar containing a
     *                                             timestamp.
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final NEDFrame frame,
            final GregorianCalendar calendar,
            final double magneticFluxDensityStandardDeviation) {
        super(magneticFluxDensity, frame, calendar);
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public StandardDeviationFrameBodyMagneticFluxDensity(final StandardDeviationFrameBodyMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets standard deviation of measured magnetic flux density expressed in
     * Teslas (T).
     *
     * @return standard deviation of measured magnetic flux density.
     */
    public double getMagneticFluxDensityStandardDeviation() {
        return mMagneticFluxDensityStandardDeviation;
    }

    /**
     * Sets standard deviation of measured magnetic flux density expressed in
     * Teslas (T).
     *
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setMagneticFluxDensityStandardDeviation(
            final double magneticFluxDensityStandardDeviation) {
        if (magneticFluxDensityStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mMagneticFluxDensityStandardDeviation =
                magneticFluxDensityStandardDeviation;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final StandardDeviationFrameBodyMagneticFluxDensity input) {
        super.copyFrom(input);

        mMagneticFluxDensityStandardDeviation = input.mMagneticFluxDensityStandardDeviation;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final StandardDeviationFrameBodyMagneticFluxDensity output) {
        super.copyTo(output);

        output.mMagneticFluxDensityStandardDeviation = mMagneticFluxDensityStandardDeviation;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), mMagneticFluxDensityStandardDeviation);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(
            final StandardDeviationFrameBodyMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(
            final StandardDeviationFrameBodyMagneticFluxDensity other,
            final double threshold) {
        return super.equals(other, threshold) &&
                Math.abs(mMagneticFluxDensityStandardDeviation
                        - other.mMagneticFluxDensityStandardDeviation) <= threshold;
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
    @Override
    public boolean equals(final FrameBodyMagneticFluxDensity other, final double threshold) {
        if (this == other) {
            return true;
        }
        if (other == null || getClass() != other.getClass()) {
            return false;
        }
        return super.equals(other, threshold);
    }

    /**
     * Checks if provided object is a
     * StandardDeviationFrameBodyMagneticFluxDensity instance
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
        final StandardDeviationFrameBodyMagneticFluxDensity other =
                (StandardDeviationFrameBodyMagneticFluxDensity) obj;
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
        final StandardDeviationFrameBodyMagneticFluxDensity result =
                (StandardDeviationFrameBodyMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
