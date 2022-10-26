// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.SplineAuto;
import frc.robot.commands.swerve.SetSwerveDriveCmd;
import frc.robot.commands.swerve.ZeroHeadingCmd;
import frc.robot.subsystems.SwerveSys;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final SwerveSys m_robotDrive = new SwerveSys();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller

  private final Joystick m_leftJoystick = new Joystick(0);
  private final Joystick m_rightJoystick = new Joystick(1);

  private final JoystickButton m_right2 = new JoystickButton(m_rightJoystick, 2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    initializeAutoChooser();
    // sc.showAll();
    // Configure default commands
   // m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new SetSwerveDrive(
        // m_robotDrive,

        // () -> -m_coDriverController.getRawAxis(1),
        // () -> -m_coDriverController.getRawAxis(0),
        // () -> -m_coDriverController.getRawAxis(4)));

        m_robotDrive.setIdleMode(true);

        m_robotDrive.setDefaultCommand(
        new SetSwerveDriveCmd(
            m_robotDrive,
            () -> m_leftJoystick.getY(),
            () -> m_leftJoystick.getX(),
            () -> m_rightJoystick.getX(),
            () -> m_leftJoystick.getRawButton(1)));

    // driver.leftTrigger.whileHeld(new JogTurnModule(
    //     m_robotDrive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3)));

    // // individual modules
    // driver.leftBumper.whileHeld(new JogDriveModule(
    //     m_robotDrive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3),
    //     true));

    // // all modules
    // driver.rightBumper.whileHeld(new JogDriveModule(
    //     m_robotDrive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3),
    //     false));     

        
    // position turn modules individually
    // driver.X_button.whenPressed(new PositionTurnModule(m_robotDrive,
    // ModulePosition.FRONT_LEFT));
    // driver.A_button.whenPressed(new PositionTurnModule(m_robotDrive,
    // ModulePosition.FRONT_RIGHT));
    // driver.B_button.whenPressed(new PositionTurnModule(m_robotDrive,
    // ModulePosition.BACK_LEFT));
    // driver.Y_button.whenPressed(new PositionTurnModule(m_robotDrive,
    // ModulePosition.BACK_RIGHT));

    m_right2.whenPressed(new ZeroHeadingCmd(m_robotDrive));
  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption("Spline", new SplineAuto(m_robotDrive));

    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void simulationPeriodic() {
    periodic();
  }

  public void periodic() {

  }

  public double getThrottle() {
    return -m_leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
