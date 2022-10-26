package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.subsystems.DriveSubsystem;

public class SplineAuto extends SequentialCommandGroup {
    public SplineAuto(DriveSubsystem driveSubsystem) {
        super(
            new ZeroHeading(driveSubsystem),
            new Spline1Path(driveSubsystem),
            new WaitCommand(0.5),
            new Spline2Path(driveSubsystem)
        );
    }
}
