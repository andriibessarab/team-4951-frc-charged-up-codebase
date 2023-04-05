    package frc.robot.commands.intake_commands;

    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj2.command.CommandBase;
    import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;

    public class Cmd_ClawIntake extends CommandBase{
        private final ClawMotorsSubsystem m_claw;
        private final Timer m_timer;

        public Cmd_ClawIntake(ClawMotorsSubsystem claw){
            m_claw = claw;
            m_timer = new Timer();
            addRequirements(m_claw);
        }

        @Override
        public void initialize() {
            m_timer.reset();
            m_timer.start();
        }

        @Override
        public void execute(){
            m_claw.spinIn();
        }

        @Override
        public boolean isFinished() {
            if (m_timer.get() > 0.5) {
                return true;
            }
            return false;
        }

        @Override
        public void end(boolean bool){
            m_claw.stop();
        }
    }
