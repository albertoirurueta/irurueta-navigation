package com.irurueta.navigation;

/**
 * Exception raised when attempting to perform an operation when not ready.
 */
@SuppressWarnings("WeakerAccess")
public class NotReadyException extends NavigationException {

    /**
     * Constructor.
     */
    public NotReadyException() {
        super();
    }

    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public NotReadyException(String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public NotReadyException(String message, Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public NotReadyException(Throwable cause) {
        super(cause);
    }
}
