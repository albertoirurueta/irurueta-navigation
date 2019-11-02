package com.irurueta.navigation.frames.converters;

import com.irurueta.navigation.frames.Frame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.units.Time;

/**
 * Converts between source and destination frames during a given time interval.
 *
 * @param <S> source frame type.
 * @param <D> destination frame type.
 */
public interface TimeIntervalFrameConverter<S extends Frame, D extends Frame> {
    /**
     * Converts source frame to a new destination frame instance.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    D convertAndReturnNew(final double timeInterval, final S source);

    /**
     * Converts source frame to a new destination frame instance.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    D convertAndReturnNew(final Time timeInterval, final S source);

    /**
     * Converts source frame to destination frame.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    void convert(final double timeInterval, final S source, final D destination);

    /**
     * Converts source frame to destination frame.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to covert to.
     */
    void convert(final Time timeInterval, final S source, final D destination);

    /**
     * Gets source frame type.
     *
     * @return source frame type.
     */
    FrameType getSourceType();

    /**
     * Gets destination frame type.
     *
     * @return destination frame type.
     */
    FrameType getDestinationType();
}
