/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

/** Sets the robot's position */
public class SetSwerveOdometryCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSys m_swerveDrive;

  private final Pose2d m_pose2d;

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   */
  public SetSwerveOdometryCmd(SwerveSys swerveDrive, Pose2d pose2d) {
    m_swerveDrive = swerveDrive;
    m_pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_swerveDrive.setOdometry(m_pose2d);
    // m_driveTrain.setNavXOffset(m_pose2d.getRotation().getDegrees());
    // if (RobotBase.isSimulation()) m_fieldSim.resetRobotPose(m_pose2d);
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
