// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//FRC9284 Version for 2023 ChargedUp game code from FRC118 EveryBot
package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants;
import edu.wpi.first.cameraserver.CameraServer;
public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSparkFront = new CANSparkMax(Constants.DriveConstants.CAN_ID_DRIVE_LEFT_FRONT, MotorType.kBrushed);
  CANSparkMax driveLeftSparkBack = new CANSparkMax(Constants.DriveConstants.CAN_ID_DRIVE_LEFT_BACK, MotorType.kBrushed);
  CANSparkMax driveRightSparkFront = new CANSparkMax(Constants.DriveConstants.CAN_ID_DRIVE_RIGHT_FRONT, MotorType.kBrushed);
  CANSparkMax driveRightSparkBack = new CANSparkMax(Constants.DriveConstants.CAN_ID_DRIVE_RIGHT_BACK, MotorType.kBrushed);
 

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on EveryBot.
   * The intake is a NEO 550 on EveryBot.
   */
  CANSparkMax arm = new CANSparkMax(Constants.ArmConstants.CAN_ID_ARM, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(Constants.IntakeConstants.CAN_ID_INTAKE, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  Joystick driver = new Joystick(0);
  Joystick operator = new Joystick(1);

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    /* Add the autonomous mode options to the chooser list */
    m_chooser.setDefaultOption("do nothing", Constants.AutoConstants.kNothingAuto);
    m_chooser.addOption("cone and mobility", Constants.AutoConstants.kConeAuto);
    m_chooser.addOption("cube and mobility", Constants.AutoConstants.kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
   CameraServer.startAutomaticCapture();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSparkFront.setInverted(false);
    driveLeftSparkBack.setInverted(false);
    
    driveRightSparkFront.setInverted(true);
    driveRightSparkBack.setInverted(true);
    
    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = (forward - turn) / (1 + Math.abs(turn));
    double right = (forward + turn) / (1 + Math.abs(turn));

    if (!(forward < Constants.DriveConstants.LOWER_FORWARD_THRESHOLD && Math.abs(turn) > Constants.DriveConstants.LOWER_TURN_THRESHOLD)){
      left *= Math.abs(left);
      right *= Math.abs(right);
     }

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftSparkFront.set(left);
    driveLeftSparkBack.set(left);
    driveRightSparkFront.set(right);
    driveRightSparkBack.set( right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }
  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  private void SetDriveTrainToBrakeMode() {
    driveLeftSparkFront.setIdleMode(IdleMode.kBrake);
    driveLeftSparkBack.setIdleMode(IdleMode.kBrake);
    driveRightSparkFront.setIdleMode(IdleMode.kBrake);
    driveRightSparkBack.setIdleMode(IdleMode.kBrake);
  }

  private void SetDriveTrainToCoastMode() {
    driveLeftSparkFront.setIdleMode(IdleMode.kCoast);
    driveLeftSparkBack.setIdleMode(IdleMode.kCoast);
    driveRightSparkFront.setIdleMode(IdleMode.kCoast);
    driveRightSparkBack.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void autonomousInit() {
    SetDriveTrainToBrakeMode();

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == Constants.AutoConstants.kConeAuto) {
      autonomousIntakePower = Constants.IntakeConstants.INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == Constants.AutoConstants.kCubeAuto) {
      autonomousIntakePower = -Constants.IntakeConstants.INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    //code to be executed in Autonomous Mode
    //this if statement does nothing and then returns to end the auto method
    if (m_autoSelected == Constants.AutoConstants.kNothingAuto) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
      return;
    }
    //the timeElapsed variable keeps a running time during autonomous for timed sequences
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    //if the ConeAuto is chosen in the Chooser, the code in this if statement will run to place a cone and drive back
    if (m_autoSelected == Constants.AutoConstants.kConeAuto) {
      //arm reaches out slowly
      if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S) {
        setArmMotor(Constants.ArmConstants.AUTO_ARM_OUTPUT_POWER);
        if(timeElapsed < 0.2){
          setIntakeMotor(-10.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        }
        //setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      }  
      //throw the cone
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S) {
        setArmMotor(Constants.ArmConstants.ARM_HOLDSTALL_POWER);
        //Set autonomousIntakePower with the correct sign to eject a cone
        setIntakeMotor(autonomousIntakePower, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } 
      //pull the arm in
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S + Constants.ArmConstants.ARM_EXTEND_TIME_S) {
        setArmMotor(-Constants.ArmConstants.ARM_OUTPUT_POWER);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } 
      //drive back a little bit to get bumpers clear for a slight turn
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S + Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_DRIVE_TIME1) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(Constants.AutoConstants.AUTO_DRIVE_SPEED, 0.0);
      } 
      //make a slight turn to compensate for drivetrain pull
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S + Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_DRIVE_TIME1+Constants.AutoConstants.AUTO_DRIVE_TIME2) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors( 0.0 ,Constants.AutoConstants.AUTO_TURN_SPEED);
      } 
      //drive backwards
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S + Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_DRIVE_TIME1+Constants.AutoConstants.AUTO_DRIVE_TIME2+Constants.AutoConstants.AUTO_DRIVE_TIME3) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(Constants.AutoConstants.AUTO_DRIVE_SPEED, 0.0);
      } 
      //turn about 180 degrees to be ready to get more game pieces in TELE
      else if (timeElapsed < Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_THROW_TIME_S + Constants.ArmConstants.ARM_EXTEND_TIME_S + Constants.AutoConstants.AUTO_DRIVE_TIME1+Constants.AutoConstants.AUTO_DRIVE_TIME2+Constants.AutoConstants.AUTO_DRIVE_TIME3+Constants.AutoConstants.AUTO_DRIVE_TIME4) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors( 0.0 ,Constants.AutoConstants.AUTO_TURN_SPEED*2);
      } 
      //Stop robot
      else {
        setArmMotor(0.0);
        setIntakeMotor(0.0, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      }
    }
   
  }

  
  int lastGamePiece;

  @Override
  public void teleopInit() {
    SetDriveTrainToCoastMode();

    lastGamePiece = Constants.GamePieceConstants.NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    double armPower;
    
    double lowerArm = operator.getRawAxis(Constants.JoystickConstants.JOYSTICK_AXIS_ID_LOWER_ARM);
    double raiseArm = operator.getRawAxis(Constants.JoystickConstants.JOYSTICK_AXIS_ID_RAISE_ARM);
    if (Math.abs(lowerArm) > Constants.ArmConstants.ARM_DEADZONE_THRESHOLD) {
      // lower the arm
      // lower arm value needs to be negative with current config
      armPower = -(Constants.ArmConstants.ARM_LOWER_MAX_OUTPUT < lowerArm ? Constants.ArmConstants.ARM_LOWER_MAX_OUTPUT : lowerArm);
    } else if (raiseArm > Constants.ArmConstants.ARM_DEADZONE_THRESHOLD) {
      // raise the arm
      armPower = Constants.ArmConstants.ARM_RAISE_MAX_OUTPUT < raiseArm ? Constants.ArmConstants.ARM_RAISE_MAX_OUTPUT : raiseArm ;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);
  
    double intakePower;
    int intakeAmps;
    if (operator.getRawButton(Constants.JoystickConstants.JOYSTICK_BUTTON_ID_CUBE_IN_OR_CONE_OUT)) {
      // cube in or cone out
      intakePower = Constants.IntakeConstants.INTAKE_OUTPUT_POWER;
      intakeAmps = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = Constants.GamePieceConstants.CUBE;
    } else if (operator.getRawButton(Constants.JoystickConstants.JOYSTICK_BUTTON_ID_CONE_IN_OR_CUBE_OUT)) {
      // cone in or cube out
      intakePower = -Constants.IntakeConstants.INTAKE_OUTPUT_POWER;
      intakeAmps = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = Constants.GamePieceConstants.CONE;
    } else if (lastGamePiece == Constants.GamePieceConstants.CUBE) {
      intakePower = Constants.IntakeConstants.INTAKE_HOLD_POWER;
      intakeAmps = Constants.IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == Constants.GamePieceConstants.CONE) {
      intakePower = -Constants.IntakeConstants.INTAKE_HOLD_POWER;
      intakeAmps = Constants.IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    setDriveMotors(driver.getRawAxis(Constants.JoystickConstants.JOYSTICK_AXIS_FORWARD)*(-1), 
    driver.getRawAxis(Constants.JoystickConstants.JOYSTICK_AXIS_TURN));
  }
}
