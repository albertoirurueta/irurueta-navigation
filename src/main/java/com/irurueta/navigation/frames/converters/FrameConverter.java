package com.irurueta.navigation.frames.converters;

import com.irurueta.navigation.frames.Frame;
import com.irurueta.navigation.frames.FrameType;

/**
 * Converts between source and destination frames.
 *
 * @param <S> source frame type.
 * @param <D> destination frame type.
 */
public interface FrameConverter<S extends Frame, D extends Frame> {
    /**
     * Converts source frame to a new destination frame instance.
     *
     * @param source source frame to convert from.
     * @return a new destination frame instance.
     */
    D convertAndReturnNew(final S source);

    /**
     * Converts source frame to destination frame.
     *
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    void convert(final S source, final D destination);

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
