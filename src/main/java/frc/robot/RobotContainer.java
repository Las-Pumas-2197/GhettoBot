// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {

  private final CommandJoystick driverJoystick = new CommandJoystick(0);
  private final differentialdrive s_Drive = new differentialdrive();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {

    s_Drive.register();

    s_Drive.runAutoBuilder();

    autoChooser.setDefaultOption("Auto 1", AutoBuilder.buildAuto("Auto1"));
    
    s_Drive.setDefaultCommand(
      new RunCommand(
        () -> s_Drive.drive(
          driverJoystick.getRawAxis(1),
          driverJoystick.getRawAxis(4),
          driverJoystick.button(6).getAsBoolean()),
        s_Drive));

      this.configureButtonBindings();
  }

  public void configureButtonBindings() {
    driverJoystick.button(1).onTrue(runOnce(() -> s_Drive.resetOdometry(new Pose2d())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
