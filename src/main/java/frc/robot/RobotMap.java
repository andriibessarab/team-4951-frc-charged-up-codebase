/**
 *This class holds all the inputs and constants used in the robot's code.
 */

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Controller;
import frc.robot.utils.Gyroscope;
import frc.robot.utils.Motor;

public class RobotMap {
	/**
	* The IDs of the Xbox controllers used for driving the robot.
	*/
	public static final int XBOX_CONTROLLER_1_ID = 0;
	public static final int XBOX_CONTROLLER_2_ID = 1;

	/**
	 * The PWM pins number of SPARK MAX motor controllers.
	 */
	public static final int REAR_LEFT_MOTOR_PWM_PIN = 0;
	public static final int REAR_RIGHT_MOTOR_PWM_PIN = 1;
	public static final int FRONT_LEFT_MOTOR_PWM_PIN = 2;
	public static final int FRONT_RIGHT_MOTOR_PWM_PIN = 3;

	/**
	 * The PWM pin number of the Limelight sensor.
	 */
	public static int LIMELIGHT_PWM_PIN; // undefined

	/**
	 * The threshold value of the controller movement below which the movement is ignored.
	 */
	public static final double CONTROLLER_THRESHOLD = 0.2;


	/**
	 * The Xbox controller objects used for driving the robot.
	 */
	public final static Controller mController1 = new Controller(RobotMap.XBOX_CONTROLLER_1_ID);
    public final static Controller mController2 = new Controller(RobotMap.XBOX_CONTROLLER_2_ID);


	/**
	 * The motor objects used for controlling the robot's drivetrain.
	 */
	public final static Motor rearLeftMotor = new Motor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN, 0, 0);
    public final static Motor rearRightMotor = new Motor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN, 0, 0);
    public final static Motor frontLeftMotor = new Motor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN, 0, 0);
    public final static Motor frontRightMotor = new Motor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN, 0, 0);

	
	/**
	 * The gyroscope object used for detecting and measuring the robot's rotation.
	 */
	public final static Gyroscope gyro = new Gyroscope();
}
