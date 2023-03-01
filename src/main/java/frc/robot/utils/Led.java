package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Spark;


/**
 * A class for controlling a Spark LED
 */
public class Led {
    /**
     * The Spark motor controller object.
     */
    private Spark led;


    /**
     * Constructs a new Led object with the specified channel.
     * Sets the LED to turn on at maximum brightness by default.
     * @param channel The PWM channel number of the Spark motor controller.
     */
    public Led(int channel) {
        // Initializes the Spark motor controller for the LED
        led = new Spark(channel);
        
        // Sets the LED to turn on at maximum brightness by default
        led.set(0.99);
    }

}
