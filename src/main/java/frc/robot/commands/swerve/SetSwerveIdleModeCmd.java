package frc.robot.commands.swerve;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveIdleModeCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSys m_swerveSys;

  private final boolean m_brake;
  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param swerveDrive The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SetSwerveIdleModeCmd(SwerveSys swerveDrive, boolean brake) {
    m_swerveSys = swerveDrive;
    m_brake = brake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSys.setIdleMode(m_brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
