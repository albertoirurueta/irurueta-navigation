package com.irurueta.navigation.frames;

import com.irurueta.navigation.NavigationException;

/**
 * Exception related to frames.
 */
public class FrameException extends NavigationException {

    /**
     * Constructor.
     */
    public FrameException() {
        super();
    }

    /**
     * Constructor with message.
     *
     * @param message message indicating the cause of the exception.
     */
    public FrameException(final String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     *
     * @param message message indicating the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public FrameException(final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     *
     * @param cause instance containing the cause of the exception.
     */
    public FrameException(final Throwable cause) {
        super(cause);
    }
}
