package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

/**
 * Put current/cone/cube in place during autonomous
 */
public class AutonomousConePlacementCommand extends CommandBase {
    private final SparkMax drivetrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final RollerIntakeSubsystem intakeSubsystem;
    private final ArmSubsystem armSubsystem;
    private final PneumaticsSubsystem pSubsystem;
    private boolean conePlaced = false;

    public AutonomousConePlacementCommand(SparkMax drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
            RollerIntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, PneumaticsSubsystem pSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.pSubsystem = pSubsystem;
        addRequirements(this.drivetrainSubsystem, this.elevatorSubsystem, this.intakeSubsystem, this.pSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        //TODO: remember to pSubsystem.move()

        // Raise elevator
        while (elevatorSubsystem.getPosition() < 10) { // specifiy how high the elevator needs to be raised
            elevatorSubsystem.raiseElevator();
        }
        elevatorSubsystem.stopElevator();

        // Intake system move cone out
        while (intakeSubsystem.getPosition() < 10) { // specifiy how far intake system move out
            intakeSubsystem.intakeOut();
        }
        intakeSubsystem.intakeStop();

        // Intake system move cone out
        while (armSubsystem.getPosition() < 10) { // specifiyat what position arm is considered open
            armSubsystem.openArm();
        }
        armSubsystem.stopArm();

        conePlaced = true;
    }

    @Override
    public boolean isFinished() {
        return conePlaced;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
