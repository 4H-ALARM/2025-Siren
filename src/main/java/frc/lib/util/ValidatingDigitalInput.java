package frc.lib.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * A digital input that provides a robustness check, and a default value if the
 * input is stuck one way or the other. This is useful for inputs that have
 * failed in one state or the other. The default value will ALWAYS be returned,
 * if the actual input state never changes. So plan accordingly! A use case is a
 * limit switch on some mechanism that may stop operating because it is
 * unplugged or gets stuck as triggered. The default value to return is NOT
 * necessarily the expected starting value, but instead the value that you wish
 * to always return when the device no longer operates. Call `rawGet` if you
 * want the real value.
 * 
 * Note: a failure state is defined as the sensor being stuck high or low since
 * the robot has started (or since being set to invalid). This does not detect
 * failures such as intermittent high or low values, or values that make no
 * sense in context (for example a high and low limit switch being triggered at
 * the same time)
 */
public class ValidatingDigitalInput implements Sendable {
    private enum State {
        initialize,
        failedTrue,
        failedFalse,
        valid
    }
    private State state = State.initialize;

    private boolean defaultValue;

    private DigitalInput realInput;

    public ValidatingDigitalInput(int channel, boolean defaultValue) {
        realInput = new DigitalInput(channel);
        this.defaultValue = defaultValue;
    }

    /**
     * Get the value of the input, which is the default value if the sensor is not yet marked as valid.
     * 
     * @return `defaultValue` if the input has not been validated yet, the real input value otherwise.
     *         false = 0V, true = 5V.
     */
    public boolean get() {
        var realValue = realInput.get();
        switch(state) {
            case initialize:
                state = realValue ? State.failedTrue : State.failedFalse;
                break;
            case failedTrue:
                if(!realValue)
                {
                    state = State.valid;
                    return realValue;
                }
                break;
            case failedFalse:
                if(realValue)
                {
                    state = State.valid;
                    return realValue;
                }
                break;
            case valid:
                return realValue;
            default:
                assert false : "Invalid state condition!";
                break;
        }
        return defaultValue;
    }

    /**
     * @return true if the state is determined to be valid (the sensor is operational)
     */
    public boolean valid() {
        return state == State.valid;
    }

    /**
     * Reset the validation state, the sensor will now return the default value
     * until it is validated.
     */
    public void invalidate() {
        state = State.initialize;
    }

    /**
     * The real input from the dio port.
     * 
     * @return The digital input object that is actually reading the io port.
     */
    public DigitalInput rawInput() {
        return realInput;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Validating Digital Input");
        builder.addBooleanProperty("Value", this::get, null);
        builder.addBooleanProperty("Valid", this::valid, null);
    }
}
