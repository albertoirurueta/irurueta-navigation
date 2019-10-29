package com.irurueta.navigation.inertial;

import com.irurueta.navigation.NavigationException;

/**
 * Base exception for all inertial related exceptions.
 */
@SuppressWarnings("WeakerAccess")
public class InertialException extends NavigationException {

    /**
     * Constructor.
     */
    public InertialException() {
        super();
    }

    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public InertialException(final String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InertialException(final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public InertialException(final Throwable cause) {
        super(cause);
    }
}
