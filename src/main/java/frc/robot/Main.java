/**
 * The main class that starts the robot.
 */ 

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;


public final class Main {
  /**
   * Private constructor to prevent instantiation of the class.
   */
  private Main() {}


  /**
   * The main method that starts the robot.
   * @param args command-line arguments passed to the program
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
