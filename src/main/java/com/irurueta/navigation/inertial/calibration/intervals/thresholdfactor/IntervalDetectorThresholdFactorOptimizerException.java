/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.navigation.NavigationException;

/**
 * Exception raised if interval detector threshold factor optimization fails for some
 * reason.
 */
public class IntervalDetectorThresholdFactorOptimizerException extends NavigationException {

    /**
     * Constructor.
     */
    public IntervalDetectorThresholdFactorOptimizerException() {
        super();
    }

    /**
     * Constructor with String containing message.
     *
     * @param message message indicating the cause of the exception.
     */
    public IntervalDetectorThresholdFactorOptimizerException(final String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     *
     * @param message message describing the cause of the exception.
     * @param cause   instance containing the cause of the exception.
     */
    public IntervalDetectorThresholdFactorOptimizerException(
            final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     *
     * @param cause instance containing the cause of the exception.
     */
    public IntervalDetectorThresholdFactorOptimizerException(final Throwable cause) {
        super(cause);
    }
}
