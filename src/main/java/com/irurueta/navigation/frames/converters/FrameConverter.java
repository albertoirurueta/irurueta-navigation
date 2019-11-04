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
