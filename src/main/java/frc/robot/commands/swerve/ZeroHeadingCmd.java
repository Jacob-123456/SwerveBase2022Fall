// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSys;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroHeadingCmd extends InstantCommand {
  private SwerveSys m_swerveSys;

  public ZeroHeadingCmd(SwerveSys drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSys = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSys.zeroHeading();
  }
}
