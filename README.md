# Team4951 FRC 2023 CHARGED UP

DriveSubsystem is based off the 
[WPILib MecanumControllerCommand example](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand)
which demonstrates trajectory generation and following with a mecanum drive using the TrajectoryGenerator and
MecanumControllerCommand classes. The default Gyro used in the example is replaced with a 
[ADIS16470_IMU](https://wiki.analog.com/first/adis16470_imu_frc). 

Our drive train uses SparkMax NEO Brushless motors ([spec sheet](https://www.revrobotics.com/rev-21-1650/)) 
and encoders and has a different characterization which should be obtained with 
[Sysid](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html).

- MecanumDriveExample is the hard coded path provided the example WPILib code 
using the PPMecanumControllerCommand.

PathPlanning uses the 3rd party
[PathPlannerLib](https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Installing).

Integrated in additional Commands for Limelight using the
[Limelight Helper API](https://github.com/LimelightVision/limelightlib-wpijava).

- Limelight passively watches for AprilTags to update pose2d
- Limelight when button pressed switches to reflector centering mode

AprilTag and Limelight 2+ performance (running PhotonVision) was mentioned
as being poor on
[Chief Delphi](https://www.chiefdelphi.com/t/terrible-photovision-performence-on-limelight-2/416749).
Limelight [Point-of-Interest](https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#point-of-interest-tracking)
is simple, it often returned missing target for upwards of 23 frames (~500ms)
making it unsuitable for alignment compared to alignment with reflective tape
which was instantaneous.

- [FRC 2023 Game Manual](https://firstfrc.blob.core.windows.net/frc2023/Manual/2023FRCGameManual.pdf)
- [FRC 2023 Check List](https://firstfrc.blob.core.windows.net/frc2023/Manual/2023-inspection-checklist.pdf)

  - R103 Must not exceed 125 lbs (~56 kg)
  - R104 Starting Configuration may not have frame perimeter greater than 120"
    or be more than 4'6" tall
  - R105 Extension Limit may not extend more than 48" beyond frame perimeter

## Troubleshooting
  - [REV Power Distribution Hub Lights](https://docs.revrobotics.com/rev-11-1850/status-led-patterns)
  - [REV SparkMax Encoder Lights](https://docs.revrobotics.com/sparkmax/status-led)