package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public final class TimerUtil {

    private final static Timer timer = new Timer(); // Timer

    // Start timer
    public final static void start() {
        timer.start();
    }

    // Reset timer
    public final static void reset() {
        timer.reset();
    }

    public final static double get() {
        return timer.get();
    }
}
