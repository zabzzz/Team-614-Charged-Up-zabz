// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  // PID Controllers and values for Drive Train PID
  ProfiledPIDController leftProfiledPIDController = 
    new ProfiledPIDController(Constants.V_kP, Constants.V_kI, Constants.V_kD,
      new Constraints(Constants.DRIVETRAIN_CONSTRAINT_VELOCITY, Constants.DRIVETRAIN_CONSTRAINT_ACCELERATION));
  ProfiledPIDController rightProfiledPIDController = 
    new ProfiledPIDController(Constants.V_kP, Constants.V_kI, Constants.V_kD,
      new Constraints(Constants.DRIVETRAIN_CONSTRAINT_VELOCITY, Constants.DRIVETRAIN_CONSTRAINT_ACCELERATION));
  double leftPIDVal;
  double rightPIDVal;
  double leftVal;
  double rightVal;

  // Create Drivetrain Motor Variables

  CANSparkMax frontRightMotor = null;
  CANSparkMax backRightMotor = null;
  CANSparkMax frontLeftMotor = null;
  CANSparkMax backLeftMotor = null;


  // Create Differntial Drive Variables
  // Differential drive is used to call arcade drive using the motors. 
  DifferentialDrive differentialDrive = null;


  public DriveTrainSubsystem() {

    // motor initalization
    frontRightMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR, MotorType.kBrushless);
    frontLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_MOTOR, MotorType.kBrushless);

    // Back motors follow front motors and invertion is set. 
    // Note: ROBOT MAY NOT GO STRAIGHT AND INVERTION MAY NEED TO CHANGE
    // MAKE SURE TO SET THE CURRENT LIMITS AS WELL
    // NOTE: BACK MOTORS are LEADERS in this example 
    frontRightMotor.follow(backRightMotor,false);
    frontLeftMotor.follow(backLeftMotor,false);

    // Current Limits Set
    frontRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT); // current limit can be changed from
    backRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT); //  40 to a higher value (max 90)
    frontLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT); // only momentarily though, just to
    backLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);  // accelerate though, just a note.

    // Create DifferentialDrive Object 
    differentialDrive = new DifferentialDrive(backLeftMotor, backRightMotor);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    leftPIDVal = moveSpeed;
    rightPIDVal = moveSpeed;

    if (rotateSpeed > 0)  {
      rightPIDVal = rightPIDVal - rotateSpeed;
      leftPIDVal = leftPIDVal + rotateSpeed;
    }
    if (rotateSpeed < 0)  {
      rightPIDVal = rightPIDVal + rotateSpeed;
      leftPIDVal = leftPIDVal - rotateSpeed;
    }
    rightVal = rightProfiledPIDController.calculate(backRightMotor.get(), rightPIDVal);
    leftVal = leftProfiledPIDController.calculate(backLeftMotor.get(), leftPIDVal);
    
    backRightMotor.set(rightVal);
    backLeftMotor.set(leftVal);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Returns rate of motor
public double getPosition(){

  double positionAverage = (Math.abs(backLeftMotor.getEncoder().getPosition()) + Math.abs(backRightMotor.getEncoder().getPosition())) / 2;

  SmartDashboard.putNumber("Drivetrain Subsystem Encoder Position", positionAverage);
  return positionAverage;
}
public void setSpeed(double val){
  backRightMotor.set(val);
  backLeftMotor.set(val);
  SmartDashboard.putNumber("Drive Left Motor Subsystem Speed Value ", backLeftMotor.get());
  SmartDashboard.putNumber("Drivetrain Right Motor Subsystem Speed Value", val);
}
public void resetEncoderValues(){
  backLeftMotor.getEncoder().setPosition(0.0);
  backRightMotor.getEncoder().setPosition(0.0);
}
public double rotateRight(double val){
  backRightMotor.set(-1*val);
  backLeftMotor.set(val);
  return Math.abs(val);
}
public double rotateLeft(double val){
  backRightMotor.set(val);
  backLeftMotor.set(-1*val);
  return Math.abs(val);
}
}
