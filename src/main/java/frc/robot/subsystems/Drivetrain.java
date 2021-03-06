/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.DrivetrainConstants;
import static frc.robot.Constants.Units;
import static frc.robot.RobotUtilities.*;

import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.DriveWrapper;
import frc.robot.UniversalVar;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private CANSparkMax[] leftMotors;
  private CANSparkMax[] rightMotors;
  private CANEncoder leftEncoder, rightEncoder;
  private CANPIDController leftLinearPid, rightLinearPid;

  private DifferentialDriveKinematics kinematicsCalc;
  private DifferentialDriveWheelSpeeds internalWheelSpeeds;
  private ChassisSpeeds internalChassis;

  private UniversalVar uniVar;
  
  private AHRS navX;
  private PIDController anglePid;

  private double leftFFVoltage = 0;
  private double rightFFVoltage = 0;

  private DriveWrapper leftDrive, rightDrive;
  private double currentTime;

  TrapezoidProfile trapProf;
  long startTime;
  
  private double leftPosition, leftVelocity, rightPosition, rightVelocity, //Grab from encoders, linear
          chassisVelocity, chassisPosition, chassisAccelleration, chassisAngle, rotVelocity; //Grab from NavX
  
  public Drivetrain(UniversalVar uniVar) {
    this.uniVar = uniVar;

    leftMotors = SetUpMotors(DrivetrainConstants.leftMotorPorts, DrivetrainConstants.inversionsLeft); //All motor stuff
    rightMotors = SetUpMotors(DrivetrainConstants.rightMotorPorts, DrivetrainConstants.inversionsRight);
    kinematicsCalc = new DifferentialDriveKinematics(DrivetrainConstants.effectiveDrivebaseWidth);

    leftEncoder = leftMotors[0].getEncoder();
    leftEncoder.setPositionConversionFactor(DrivetrainConstants.positionConversionFactor); //Converts to meters and meters/sec
    leftEncoder.setVelocityConversionFactor(DrivetrainConstants.velocityConversionFactor);
    leftDrive = new DriveWrapper(DrivetrainConstants.Ks[0], DrivetrainConstants.Kv[0], DrivetrainConstants.Ka[0]);

    rightEncoder = rightMotors[0].getEncoder();
    rightEncoder.setVelocityConversionFactor(DrivetrainConstants.velocityConversionFactor);
    rightEncoder.setPositionConversionFactor(DrivetrainConstants.positionConversionFactor);
    rightDrive = new DriveWrapper(DrivetrainConstants.Ks[1], DrivetrainConstants.Kv[1], DrivetrainConstants.Ka[1]);

    leftLinearPid = new CANPIDController(leftMotors[0]);
    rightLinearPid = new CANPIDController(rightMotors[0]);

    setupPID(leftLinearPid, leftFFVoltage, true);
    setupPID(rightLinearPid, rightFFVoltage, false);
    
    navX = new AHRS(SPI.Port.kMXP);
    anglePid = new PIDController(DrivetrainConstants.AngleKp, 
                                  DrivetrainConstants.AngleKi,
                                  DrivetrainConstants.AngleKd);
    anglePid.setTolerance(DrivetrainConstants.AngleTolerance);
  }

  public void startUp(){
    for(CANSparkMax i : leftMotors){
      i.set(0);
    }
    for(CANSparkMax i : rightMotors){
      i.set(0);
    }
  }

  void setupPID(CANPIDController linearPID, double FF, boolean isLeft){
    int index = isLeft ? 0:1;

    linearPID.setP(DrivetrainConstants.Kp[index], 0);
    linearPID.setI(DrivetrainConstants.Ki[index], 0);
    linearPID.setD(DrivetrainConstants.Kd[index], 0);
    linearPID.setIZone(DrivetrainConstants.Izone[index], 0);
    linearPID.setOutputRange(DrivetrainConstants.Min[index], DrivetrainConstants.Max[index], 0);
  }

  public void setDrive(double leftSpeed, double rightSpeed, Units lengthUnit){ //Pure differential drive
    switch (lengthUnit) { //Permits the use of all kinds of units.  Internally still working with m/s though
      case INCHES:
        leftSpeed = leftSpeed /  39.3701;
        rightSpeed = rightSpeed /  39.3701;
        break;
      case ROTATIONS:
        leftSpeed = leftSpeed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        rightSpeed = rightSpeed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        break;
      case PERCENT:
        leftSpeed = leftSpeed * DrivetrainConstants.maximumVelocity;
        rightSpeed = rightSpeed * DrivetrainConstants.maximumVelocity;
        break;
    }
    // currentTime = System.currentTimeMillis() * 1000;

    leftFFVoltage = leftDrive.calculateFeedForward(leftSpeed);
    leftLinearPid.setReference(0, ControlType.kVelocity, 0, leftFFVoltage);
    rightFFVoltage = rightDrive.calculateFeedForward(rightSpeed);
    rightLinearPid.setReference(0, ControlType.kVelocity, 0, rightFFVoltage);

    // leftSpeed = leftSpeed / DrivetrainConstants.maximumVelocity;
    // rightSpeed = rightSpeed / DrivetrainConstants.maximumVelocity;
    // leftMotors[0].set(leftSpeed);
    // rightMotors[0].set(rightSpeed);
  }

  public void setDrive (double leftSpeed, double rightSpeed, boolean revDrive, Units inUnits) { //Reversable drive wrapper
    if (revDrive) {
      this.setDrive(-rightSpeed, -leftSpeed, inUnits);
    }
  }

  //This version just uses Chassis classes to convert from velocity and rotational terms to two velocity terms, 
  //and passes them to the OG setDrive.
  public void setDrive (double speed, double rotationalVelocity, Units lengthUnit, Units rotationUnit) { 
    switch (lengthUnit) { //Converts to m/s
      case INCHES:
        speed = speed / 39.3701;
        break;
      case ROTATIONS:
        speed = speed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        break;
    } switch (rotationUnit) { //Converts to rad/sec
      case ROTATIONS:
        rotationalVelocity = rotationalVelocity * (Math.PI * 2);
        break;
      case DEGREES:
        rotationalVelocity = rotationalVelocity * (Math.PI / 180);
        break;
    }

    internalChassis = new ChassisSpeeds(speed, 0, rotationalVelocity); //These simply convert between the units to wheel speeds
    internalWheelSpeeds = kinematicsCalc.toWheelSpeeds(internalChassis);

    this.setDrive(internalWheelSpeeds.leftMetersPerSecond, //And this just passes it through
      internalWheelSpeeds.rightMetersPerSecond, 
      Units.METERS);
  }

  public void trajectoryInit(){
    TrajectoryConfig config = new TrajectoryConfig(0.75, 10);
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    Pose2d startTraj = new Pose2d(1.54, 23.23, Rotation2d.fromDegrees(-180));
    Pose2d end = new Pose2d(23.7, 6.8, Rotation2d.fromDegrees(-160));
    
    points.add(new Translation2d(14, 23));
    points.add(new Translation2d(21, 18));
    var trajectory = TrajectoryGenerator.generateTrajectory(startTraj, points, end, config);
  }

  public void trapezoidProfilingInit(double distance){
    Constraints newConstraints = new Constraints(0.75, 10); // Accel: m/s^2
    State goal = new State(distance, 0);
    State start = new State(0, this.leftVelocity);
    trapProf = new TrapezoidProfile(newConstraints, goal, start);
    startTime = System.currentTimeMillis() * 1000;
  }

  public boolean trapezoidProfiling(){
    long passedTime = System.currentTimeMillis()*1000-startTime;
    double trapezoidVel = trapProf.calculate(passedTime).velocity;
    setDrive(trapezoidVel, trapezoidVel, Units.METERS);
    return trapProf.isFinished(passedTime);
  }
  
  public void turnToAngleInit(double angleInDegrees){ //This should be called ONCE, to move an amount relative to current angle
    anglePid.setSetpoint(navX.getAngle() + angleInDegrees);
  }

  public boolean reachedAngle(){ //This does the actual work of angle PID
    double pidVal = MathUtil.clamp(anglePid.calculate(navX.getAngle()), -1, 1);
    setDrive(pidVal, pidVal, Units.METERS);
    return anglePid.atSetpoint();
  }

  public double getAngularVelocity(){
    return this.rotVelocity;
  } public double getChassisVelocity(){
    return this.chassisVelocity;
  } public double getAngle(){
    return this.chassisAngle;
  } public double getAcceleration(){
    return 0;
  } public double getLeftSpeed() {
    return this.leftVelocity;
  } public double getLeftPosition(){
    return this.leftPosition;
  } public double getRightSpeed(){
    return this.rightVelocity;
  } public double getRightPosition() {
    return this.rightPosition;
  }

  public void grabSensors() {
    this.leftPosition = this.leftEncoder.getPosition();
    this.leftVelocity = this.leftEncoder.getVelocity();
    this.rightPosition = this.rightEncoder.getPosition();
    this.rightVelocity = this.rightEncoder.getVelocity();
    
    this.chassisAngle = this.navX.getAngle();
    this.chassisPosition = this.navX.getDisplacementX();
    this.chassisVelocity = this.navX.getVelocityX();
    this.chassisAccelleration = this.navX.getRawAccelX();
    this.rotVelocity = this.navX.getRawGyroZ();

    uniVar.add("Drivetrain-LeftPos", leftPosition);
    uniVar.add("Drivetrain-LeftVel", leftVelocity);
    uniVar.add("Drivetrain-RightPos", rightPosition);
    uniVar.add("Drivetrain-RightVel", rightVelocity);
    uniVar.add("Drivetrain-ChassisAngle", chassisAngle);
    uniVar.add("Drivetrain-ChassisPosition", chassisPosition);
    uniVar.add("Drivetrain-ChassisAccelleration", chassisAccelleration);
    uniVar.add("Drivetrain-RotVel", rotVelocity);
  }

  @Override
  public void periodic() {
    grabSensors();
    SmartDashboard.putNumber("Left Speed", this.leftVelocity);
    SmartDashboard.putNumber("Right Speed", this.rightVelocity);
  }
}
