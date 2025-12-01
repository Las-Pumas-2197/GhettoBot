// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
public class RobotContainer {

  //stuff n things
  private final CommandJoystick driverJoystick = new CommandJoystick(0);
  private final differentialdrive s_Drive = new differentialdrive();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final PathConstraints constraints = new PathConstraints(3, 3, 1*Math.PI, 1*Math.PI);
  private SparkMax leftWheel = new SparkMax(12, MotorType.kBrushed);
  private SparkMax rightWheel = new SparkMax(13, MotorType.kBrushed);
  private Pose2d targetpose = new Pose2d(7, 2, new Rotation2d());
  private final Vision m_vision = new Vision();

  public RobotContainer() {

    //maybe needed??? idk check roboRIO log
    s_Drive.register();

    //set pathfinder type
    Pathfinding.setPathfinder(new LocalADStar());

    //configure autobuilder
    s_Drive.runAutoBuilder();

    //configure autochooser with loaded autos
    autoChooser.setDefaultOption("Auto 1", AutoBuilder.buildAuto("Auto1"));

    //set default drive command when no commands are scheduled
    s_Drive.setDefaultCommand(
      new RunCommand(
        () -> s_Drive.drive(
          driverJoystick.getRawAxis(1),
          driverJoystick.getRawAxis(4),
          driverJoystick.button(6).getAsBoolean()),
        s_Drive));

    //configure bindings
    this.configureButtonBindings();

    // vision
    
  }
  
  
  //put bindings here, duh
  public void configureButtonBindings() {
    driverJoystick.button(1).onTrue(runOnce(() -> s_Drive.resetOdometry(new Pose2d())));
    driverJoystick.button(2).onTrue(pathfind(targetpose).andThen(pathfind(new Pose2d(3, 3, new Rotation2d()))));
    driverJoystick.button(4).onTrue(runOnce(() -> s_Drive.getCurrentCommand().cancel()));
    driverJoystick.povUp().whileTrue(runEnd(() -> forward(), () -> stop()));
    driverJoystick.povDown().whileTrue(runEnd(() -> backwards(), () -> stop()));
  }

  public Command pathfind(Pose2d pose) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0);
      pathfindingCommand.addRequirements(this.s_Drive);
      return pathfindingCommand;
  }

  //format autochooser command here for Robot.java
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  private void forward(){
    leftWheel.set(1);
    rightWheel.set(-1);
  }
  private void backwards(){
    leftWheel.set(-1);
    rightWheel.set(1);
  }
  private void stop(){
    leftWheel.set(0);
    rightWheel.set(0);
  }
}
