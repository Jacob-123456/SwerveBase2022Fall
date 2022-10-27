package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.paths.Spline1Path;
import frc.robot.commands.auto.paths.Spline2Path;
import frc.robot.commands.swerve.ZeroHeadingCmd;
import frc.robot.subsystems.SwerveSys;

public class SplineAuto extends SequentialCommandGroup {
    public SplineAuto(SwerveSys swerveSys) {
        super(
            new ZeroHeadingCmd(swerveSys),
            new Spline1Path(swerveSys),
            new WaitCommand(0.5),
            new Spline2Path(swerveSys)
        );
    }
}
