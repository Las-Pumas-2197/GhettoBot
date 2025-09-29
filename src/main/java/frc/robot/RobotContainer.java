// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Joystick driverJoystick = new Joystick(0);
  private final differentialdrive s_Drive = new differentialdrive();


  public RobotContainer() {
  }

  public void teleop() {
    s_Drive.drive(driverJoystick.getRawAxis(1), driverJoystick.getRawAxis(4), driverJoystick.getRawButton(6), driverJoystick.getRawButton(1));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
