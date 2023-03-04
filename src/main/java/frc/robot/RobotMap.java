package frc.robot;

/**
 * This class holds all the inputs and constants used in the robot's code.
 */
public final class RobotMap {

	// The IDs of the Xbox controllers used for driving the robot
	public static final int XBOX_DRIVER_CONTROLLER_ID = 0;
	public static final int XBOX_ARM_CONTROLLER_ID = 1;

	// The PWM pins number of SPARK MAX motor controllers.
	public static final int FRONT_RIGHT_MOTOR_PWM_PIN = 1;
	public static final int FRONT_LEFT_MOTOR_PWM_PIN = 2;
	public static final int REAR_RIGHT_MOTOR_PWM_PIN = 3;
	public static final int REAR_LEFT_MOTOR_PWM_PIN = 4;

	public static final int MOTOR_SPEED_MULTIPLIER = 1;

	// ELEVATOR SUBSYSTEM
	public static final int ELEVATOR_PWM_PIN = 5;
	public static final double ELEVATOR_SPEED_MULTIPLIER = 0.5;

	// ARM SUBSYSTEM
	public static final int INTAKE_PWM_PIN = 6;
	public static final double INTAKE_SPEED_MULTIPLIER = 0.5;

	// ARM SUBSYSTEM
	public static final int ARM_PWM_PIN = 7;
	public static final double ARM_SPEED_MULTIPLIER = 0.5;

	// Liemlight Hostname
	public static final String LIMELIGHT_HOSTNAME = "limelight-cdslime";

	// Camera device inputs and resolution constants
	public static final int CAMERA_FRONT_DEV = 0;
	public static final int CAMERA_INTAKE_DEV = 1;
	public static final int CAMERA_RES_H = 160;
	public static final int CAMERA_RES_W = 283;

	// Led Channel
	public static final int LED_CHANNEL = 0;
}
