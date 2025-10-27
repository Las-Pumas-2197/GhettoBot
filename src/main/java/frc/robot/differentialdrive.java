// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class differentialdrive extends SubsystemBase {

  //instances
  private final SparkMax FL = new SparkMax(configuration.c_FLcanID, MotorType.kBrushless);
  private final SparkMax RL = new SparkMax(configuration.c_RLcanID, MotorType.kBrushless);
  private final SparkMax FR = new SparkMax(configuration.c_FRcanID, MotorType.kBrushless);
  private final SparkMax RR = new SparkMax(configuration.c_RRcanID, MotorType.kBrushless);
  private final RelativeEncoder enc_L = FL.getEncoder();
  private final RelativeEncoder enc_R = FR.getEncoder();
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DifferentialDriveKinematics kine_drive = new DifferentialDriveKinematics(configuration.c_drivetrackwidth);
  private final DifferentialDriveOdometry odo_drive = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(-gyro.getAngle())), enc_L.getPosition(), enc_R.getPosition());
  private RobotConfig robotConfig;

  private final Field2d field = new Field2d();

  //global variables
  private double d_drivespeedmult;
  private double d_leftspeed;
  private double d_rightspeed;

  public differentialdrive() {
    
    //configure spark maxes
    FL.configure(configuration.cfg_frontleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RL.configure(configuration.cfg_rearleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FR.configure(configuration.cfg_frontright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RR.configure(configuration.cfg_rearright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //calibrate gyroscope
    gyro.calibrate();

    //declare robot config from path planner gui
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    //zero encoders at start of code
    enc_L.setPosition(0);
    enc_R.setPosition(0);
  }

  public void runAutoBuilder() {
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                     // ChassisSpeeds. Also optionally outputs individual module
                                                     // feedforwards
        new PPLTVController(
          0.020,
          configuration.c_drivespeedmax
        ),
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void drive(double Xspeed, double Zspeed, boolean turbo_mode){

    //drive speed multiplier for controllability
    d_drivespeedmult = turbo_mode ? 1 : 0.25;

    //compose drive speeds
    ChassisSpeeds speeds = new ChassisSpeeds(Xspeed  * d_drivespeedmult * (-1), 0, Zspeed * (-1));
    DifferentialDriveWheelSpeeds wheelspeeds = kine_drive.toWheelSpeeds(speeds);

    //pass the speeds to persistent variables
    d_leftspeed = wheelspeeds.leftMetersPerSecond;
    d_rightspeed = wheelspeeds.rightMetersPerSecond;

    //apply volts to motors
    RL.setVoltage(wheelspeeds.leftMetersPerSecond * 12);
    RR.setVoltage(wheelspeeds.rightMetersPerSecond * 12);
  }

  public Pose2d getPose() {
    return odo_drive.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odo_drive.resetPose(pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kine_drive.toChassisSpeeds(new DifferentialDriveWheelSpeeds(enc_L.getVelocity(), enc_R.getVelocity()));
  }

  public void autoDrive(ChassisSpeeds speeds) {
    //speeds.omegaRadiansPerSecond *= -1;
    DifferentialDriveWheelSpeeds wheelspeeds = kine_drive.toWheelSpeeds(speeds);

    //pass the speeds to persistent variables
    d_leftspeed = wheelspeeds.leftMetersPerSecond;
    d_rightspeed = wheelspeeds.rightMetersPerSecond;

    //apply volts to motors, convert to [-1, 1] interval and mult by 12 to convert to volts
    RL.setVoltage((wheelspeeds.leftMetersPerSecond / configuration.c_drivespeedmax) * 12);
    RR.setVoltage((wheelspeeds.rightMetersPerSecond / configuration.c_drivespeedmax)* 12);
  }

  @Override
  public void periodic() {

    //update field pose
    field.setRobotPose(this.getPose());

    //update odometry
    odo_drive.update(new Rotation2d(Math.toRadians(-gyro.getAngle())), enc_L.getPosition(), enc_R.getPosition());

    //telemetry
    SmartDashboard.putNumber("Xpos", odo_drive.getPoseMeters().getX());
    SmartDashboard.putNumber("Ypos", odo_drive.getPoseMeters().getY());
    SmartDashboard.putNumber("Left Speed", d_leftspeed);
    SmartDashboard.putNumber("Right Speed", d_rightspeed);
    SmartDashboard.putNumber("Drive Speed Mult", d_drivespeedmult);
    SmartDashboard.putNumber("RightPos", enc_R.getPosition());
    SmartDashboard.putNumber("LeftPos", enc_L.getPosition());
    SmartDashboard.putData("odometry field", field);
  }
}
