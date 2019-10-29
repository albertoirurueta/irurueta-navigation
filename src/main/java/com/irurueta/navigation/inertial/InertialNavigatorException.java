package com.irurueta.navigation.inertial;

/**
 * Exception raised when inertial navigation fails for some reason (typically, numerical instabilities).
 */
@SuppressWarnings("ALL")
public class InertialNavigatorException extends InertialException {

    /**
     * Constructor.
     */
    public InertialNavigatorException() {
        super();
    }

    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public InertialNavigatorException(final String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InertialNavigatorException(final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public InertialNavigatorException(final Throwable cause) {
        super(cause);
    }
}
