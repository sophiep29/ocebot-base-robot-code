// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ControllerConfig;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain drivetrain = new Drivetrain();
  // changes speed when picking up from source
  private double speedMultiplier = 1.0;
  private boolean fieldRelative = true;
  private AutoChooser autoChooser = new AutoChooser();
  @Logged public boolean autoDisabled = true;
  private Field2d selectedSpot = new Field2d();

  public RobotContainer() {
    // how we selected where we were going to score
    DataLogManager.start();
    configureBindings();
    SmartDashboard.putData("scheduler", CommandScheduler.getInstance());
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.drive(
                    applyDeadband(-controller.getLeftY() * speedMultiplier),
                    applyDeadband(-controller.getLeftX() * speedMultiplier),
                    applyDeadband(-controller.getRightX() * speedMultiplier),
                    fieldRelative,
                    true),
            drivetrain));
  }

  // deadbands for driving
  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  // autonomous command
  public Command getAutonomousCommand() {
    return Commands.sequence();
  }
}
