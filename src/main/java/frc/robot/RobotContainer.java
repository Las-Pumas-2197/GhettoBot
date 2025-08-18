// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.zip.ZipEntry;

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private double actualHeading = 0;
  private double desiredHeading = 0;
  private final SparkMax FL = new SparkMax(configuration.c_FLcanID, MotorType.kBrushless);
  private final SparkMax RL = new SparkMax(configuration.c_RLcanID, MotorType.kBrushless);
  private final SparkMax FR = new SparkMax(configuration.c_FRcanID, MotorType.kBrushless);
  private final SparkMax RR = new SparkMax(configuration.c_RRcanID, MotorType.kBrushless);
  private final RelativeEncoder enc_L = FL.getEncoder();
  private final RelativeEncoder enc_R = FR.getEncoder();
  private final Joystick driverJoystick = new Joystick(0);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDriveKinematics kine_drive = new DifferentialDriveKinematics(configuration.c_drivetrackwidth);

  //private final DifferentialDriveOdometry odo_drive = new DifferentialDriveOdometry(null, null, null);

  private final SlewRateLimiter slew_drivespeedmult = new SlewRateLimiter(1);
 // private final TrapezoidProfile.Constraints headingConstraints = new TrapezoidProfile.Constraints(1*Math.PI, 25*Math.PI);
  private final PIDController headingController = new PIDController(1, 0, 0);
  private double d_drivespeedmult;

  public RobotContainer() {
    FL.configure(configuration.cfg_frontleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RL.configure(configuration.cfg_rearleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FR.configure(configuration.cfg_frontright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RR.configure(configuration.cfg_rearright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    gyro.calibrate();
    configureBindings();
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    d_drivespeedmult = 0;
  }

  private void configureBindings() {


  }
  public void drive(){
    //actualHeading = MathUtil.angleModulus(((gyro.getAngle()/180)* Math.PI));
    //desiredHeading = driverJoystick.getDirectionRadians();
    //if(desiredHeading == Math.PI){
    //  desiredHeading = actualHeading;
    //}
    //SmartDashboard.putNumber("Actual Heading", actualHeading);
    //SmartDashboard.putNumber("Desired Heading", desiredHeading);
    d_drivespeedmult = driverJoystick.getRawButton(6) ? 1 : 0.25;
    if(driverJoystick.getRawButton(1)){
      enc_R.setPosition(0);
      enc_L.setPosition(0);
    }
    ChassisSpeeds speeds = new ChassisSpeeds(driverJoystick.getRawAxis(1)  * d_drivespeedmult * (-1), 0, driverJoystick.getRawAxis(4) * (-1));
    //ChassisSpeeds speeds = new ChassisSpeeds((-1)* driverJoystick.getMagnitude() * slew_drivespeedmult.calculate(d_drivespeedmult) * Math.cos(actualHeading-desiredHeading), 0, headingController.calculate(actualHeading, desiredHeading));
    //driverJoystick.getRawAxis(1) * slew_drivespeedmult.calculate(d_drivespeedmult)
    DifferentialDriveWheelSpeeds wheelspeeds = kine_drive.toWheelSpeeds(speeds);

    //Telelmety
    SmartDashboard.putNumber("X speed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Rotation", speeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Left Speed", wheelspeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed", wheelspeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Drive Speed Mult", d_drivespeedmult);
    // Encoder Telemetry
    SmartDashboard.putNumber("RightPos", enc_R.getPosition());
    SmartDashboard.putNumber("LeftPos", enc_L.getPosition());
    RL.setVoltage(wheelspeeds.leftMetersPerSecond * 12);
    RR.setVoltage(wheelspeeds.rightMetersPerSecond * 12);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
