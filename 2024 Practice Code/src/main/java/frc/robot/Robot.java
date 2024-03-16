// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.event.EventLoop;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import java.lang.Math;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cscore.MjpegServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kMoveForward = "Only Move";
  private static final String kMoveLeft = "Move forward left";
  private static final String kMoveRight = "Move forward right";
  private static final String kZigZag = "Zigzag Movement";
  private String m_selected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final int leftLeaderDeviceID = 14;
  private static final int leftFollowerDeviceID = 10;
  private static final int rightLeaderDeviceID = 13;
  private static final int rightFollowerDeviceID = 12;
  /*Ids to use on the practice bot
  private static final int leftLeaderDeviceID = 12;
  private static final int leftFollowerDeviceID = 13;
  private static final int rightLeaderDeviceID = 10;
  private static final int rightFollowerDeviceID = 14;*/
  private static final int elevatorMotorDeviceID = 15;
  private static final int climbMotorDeviceID = 16;
  private final DigitalInput climbLimitSwitchDown1 = new DigitalInput(0);
  private final DigitalInput climbLimitSwitchDown2 = new DigitalInput(1);
  private final DigitalInput climbLimitSwitchUp = new DigitalInput(2);
  private static final int topShooterMotorDeviceID = 18;
  private static final int bottomShooterMotorDeviceID = 19;
  private CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower, climbMotor;
  private CANSparkMax topShooterMotor, bottomShooterMotor;
  private DifferentialDrive driveBase;
  private DifferentialDrive shooterBase;
  private XboxController driveController;
  private XboxController operatorController;
  private static final double shooterWheelCirc = Math.PI * .1;
  private static final double climbWheelCirc = Math.PI * .02;
  private static final int countsPerRev = 42;
  private static final double driveWheelscirc = .26;
  private static final double climbSpeed = 0.35;
  // creates an EventLoop object which checks for the inputs it's bound to (e.g. rightTrigger(loop))
  private static final EventLoop loop = new EventLoop();
  private RelativeEncoder shooterEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder climberEncoder;
  //private double meterConversionFactor = 0.03125;
  private double chassisPosition;

  /*NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");

  //Networktables values from Limelight to read.
  private double x; 
  private double y;
  private double area;*/

  public Timer timer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
  UsbCamera cam1 = CameraServer.startAutomaticCapture("Drive Camera", 0);
  cam1.setResolution(160, 120);
  cam1.setFPS(30);
  try (MjpegServer mjpegServer = new MjpegServer("Serve_USB Camera 0", 1181)) {
    mjpegServer.setSource(cam1);
  }
  m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Move Forward", kMoveForward);
    m_chooser.addOption("Move forward left", kMoveLeft);
    m_chooser.addOption("Move forward right", kMoveRight);
    m_chooser.addOption("Zigzag Movement", kZigZag);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftLeader = new CANSparkMax(leftLeaderDeviceID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(leftFollowerDeviceID, MotorType.kBrushless);
    rightLeader = new CANSparkMax(rightLeaderDeviceID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(rightFollowerDeviceID, MotorType.kBrushless);
    climbMotor = new CANSparkMax(climbMotorDeviceID, MotorType.kBrushless);
    //elevatorMotor = new CANSparkMax(elevatorMotorDeviceID, MotorType.kBrushless);
    topShooterMotor = new CANSparkMax(topShooterMotorDeviceID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkMax(bottomShooterMotorDeviceID, MotorType.kBrushless);

    climbMotor.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);

    topShooterMotor.setOpenLoopRampRate(0);
    bottomShooterMotor.setOpenLoopRampRate(0);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    driveBase = new DifferentialDrive(leftLeader, rightLeader);
    shooterBase = new DifferentialDrive(topShooterMotor,bottomShooterMotor);

    shooterEncoder = topShooterMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, countsPerRev);
    shooterEncoder.setPositionConversionFactor((10 * shooterWheelCirc) / countsPerRev);

    climberEncoder = climbMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, countsPerRev);
    climberEncoder.setPositionConversionFactor((10 * climbWheelCirc) / countsPerRev);

    leftEncoder = leftLeader.getEncoder(SparkRelativeEncoder.Type.kHallSensor, countsPerRev);
    leftEncoder.setPositionConversionFactor((10 * driveWheelscirc) / countsPerRev);

    rightEncoder = rightLeader.getEncoder(SparkRelativeEncoder.Type.kHallSensor, countsPerRev);
    rightEncoder.setPositionConversionFactor((10 * driveWheelscirc) / countsPerRev);

    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    timer = new Timer();

    SmartDashboard.putNumber("Shooter distance in cm:", shooterEncoder.getPosition());
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_selected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_selected);
     shooterEncoder.setPosition(0);
     leftEncoder.setPosition(0);
     rightEncoder.setPosition(0);
     climberEncoder.setPosition(0);
     timer.reset();
     timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*//read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);*/

    // lower the climber when in autonomous until it hits the limit switches
    // ---- all limit switches are false by default ----
    if(climbLimitSwitchDown1.get() == false || climbLimitSwitchDown2.get() == false ){
      climbMotor.set(0);
    } else {
      climbMotor.set(-climbSpeed);
    }

    switch (m_selected) {
      case kMoveForward:
        // Shoot and move forward 320 cm\
          autonomousShootLogic();
        
        // drive base positioning
        if(leftEncoder.getPosition() <= 96){
          leftLeader.set(-0.1);
          rightLeader.set(0.1);
           driveBase.feed();
        } else if(leftEncoder.getPosition() > 96 && leftEncoder.getPosition() <= 126){
          leftLeader.set(-0.1);
          rightLeader.set(-0.1);
          driveBase.feed();
        } else if(leftEncoder.getPosition() > 126 && leftEncoder.getPosition() <= 212) {
          leftLeader.set(-0.1);
          rightLeader.set(0.1);
           driveBase.feed();
        } else {
          leftLeader.set(0);
          rightLeader.set(0);
           driveBase.feed();
        } 
        break;

        case kMoveLeft: 
          autonomousShootLogic();
        if(leftEncoder.getPosition() <= 96){
          leftLeader.set(0.1);
          rightLeader.set(-0.1);
        } else if(leftEncoder.getPosition() > 96 && leftEncoder.getPosition() <= 106){
          leftLeader.set(0);
          rightLeader.set(-0.25);
        } else if(leftEncoder.getPosition() > 106 && leftEncoder.getPosition() <= 202){
          leftLeader.set(0.1);
          rightLeader.set(-0.1);
        } else{
          leftLeader.set(0);
          rightLeader.set(0);
        }
        break;

        case kMoveRight:
          //---o---o---o---o--- start shooter ---o---o---o---o---
          autonomousShootLogic();
          //----o----o---o---o--- end shooter ----o----o----o----o---
        if(leftEncoder.getPosition() <= 96){
          leftLeader.set(0.1);
          rightLeader.set(-0.1);
        } else if(leftEncoder.getPosition() > 96 && leftEncoder.getPosition() <= 106){
          leftLeader.set(0.25);
          rightLeader.set(0);
        } else if(leftEncoder.getPosition() > 106 && leftEncoder.getPosition() <= 202){
          leftLeader.set(0.1);
          rightLeader.set(-0.1);
        } else{
          leftLeader.set(0);
          rightLeader.set(0);
        }
        break;

        case kZigZag:
          //::::::::::::::::  start shooter ::::::::;
          autonomousShootLogic();
          //---------- end shooter ----------- 
        if(leftEncoder.getPosition() <= 32){
          leftLeader.set(0.1);
          rightLeader.set(-0.1);
        } else if(leftEncoder.getPosition() > 32 && leftEncoder.getPosition() <= 37){
          leftLeader.set(0);
          rightLeader.set(-0.25);
        } else if(leftEncoder.getPosition() > 37 && leftEncoder.getPosition() <= 69){
          leftLeader.set(0.15);
          rightLeader.set(-0.15);
        } else{
          leftLeader.set(0);
          rightLeader.set(0);
        }
        break;
        }
        //post to smart dashboard periodically
        SmartDashboard.putNumber("Chassis distance in m",leftEncoder.getPosition());
        /*SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);*/
      // case kDefaultAuto:

      // default:
        // Put default auto code here
       // break;
       
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
     leftEncoder.setPosition(0);
     rightEncoder.setPosition(0);
     climberEncoder.setPosition(0);
     shooterEncoder.setPosition(0);
  }

  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // checks for every button binding the loop object is bound to.
    loop.poll();
    // Button bindings for the operator stick
    if (operatorController.getRawButtonPressed(XboxController.Axis.kRightTrigger.value)){
      timer.start();
    }else if (operatorController.getRawButtonReleased(XboxController.Axis.kRightTrigger.value)){
     timer.stop();
     timer.reset();
    }


    /*fires the shooter. Left trigger takes notes in, right pressed lightly fires top shooter
    pressing right trigger heavily fires the bottom shooter. Light press into a heavy press shoots correctly*/
    if(operatorController.leftTrigger(loop).getAsBoolean()){
      topShooterMotor.set(-0.4);
      bottomShooterMotor.set(-0.4);
    } else if(operatorController.getRightTriggerAxis()>=0.25){
      topShooterMotor.set(1); 
      if(operatorController.getRightTriggerAxis()>=0.9){
      bottomShooterMotor.set(1);
      }
    } else if(operatorController.getRightTriggerAxis() == 0 || operatorController.getLeftTriggerAxis() == 0) {
        topShooterMotor.set(0);
        bottomShooterMotor.set(0);
    }

    //Elevator and Basket controls
    /*if (operatorController.getLeftY()<-0.5) {
      elevatorMotor.set(0.2);
    } else if (operatorController.getLeftY()>0.5) {
      elevatorMotor.set(-0.2);
    } */

    // handles the logic for the climber
    climbLogic();
   
    
    //Button bindings for the drive controller
    if (driveController.rightBumper(loop).getAsBoolean()) {
      driveBase.tankDrive(LeftThrottle(), -LeftThrottle());
    } else if(!driveController.leftBumper(loop).getAsBoolean()) {
      driveBase.tankDrive(LeftThrottle(), RightThrottle());
    }
    
    driveBase.feed();
    shooterBase.feed();

    chassisPosition = (leftEncoder.getPosition() - rightEncoder.getPosition())/2;
    SmartDashboard.putNumber("Chassis distance in m",chassisPosition);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public double RightThrottle() {
    double throttleInput = driveController.getRightY() ;
      //return (Math.pow(throttleInput,5))/3  + (Math.pow(throttleInput,3))/3 +throttleInput/3;
      return (Math.pow(throttleInput,3))/3 +throttleInput/3;
      //  return throttleInput;
  }

  public double LeftThrottle() {
    double throttleInput = -driveController.getLeftY();
     // return (Math.pow(throttleInput,5))/3  + (Math.pow(throttleInput,3))/3 +throttleInput/3;
      return (Math.pow(throttleInput,3))/3 +throttleInput/3;
 //   return throttleInput;
  }

  
  // handles all of the logic for climbing for the robot
  public void climbLogic()
  {

    // check if the operator wants to move the climber up
    if(operatorController.y(loop).getAsBoolean() == true)
    {
      // check upper limits before moving the climber up
      if(climbLimitSwitchUp.get() == false) {
        // upper limit switch is pressed, we don't want the climber to move
        climbMotor.set(0);
      }
      else
      {
        // set climber to move up
        climbMotor.set(climbSpeed);
      }
    }

    // check if the opeator wants to move the climber down
    else if(operatorController.a(loop).getAsBoolean() == true)
    {
      // check lower limits before moving the climber down
      if(climbLimitSwitchDown1.get() == false || climbLimitSwitchDown2.get() == false)
      {
        // don't move the climb motor if already at lowest position
        climbMotor.set(0);
      }
      else
      {
        // set climber to move down
        climbMotor.set(-climbSpeed);
      }
    }

    // the operator doesn't want to move the climber
    else
    {
      climbMotor.set(0);
    }

    // print limit switch values to the dashboard for debugging
    SmartDashboard.putBoolean("Up Limit Switch", climbLimitSwitchUp.get());
    SmartDashboard.putBoolean("Down Limit Switch 1", climbLimitSwitchDown1.get());
    SmartDashboard.putBoolean("Down Limit Switch 2", climbLimitSwitchDown2.get());
  } // end climbLogics

  // auto shoot
  public void autonomousShootLogic()
  {
    // default motor values
    topShooterMotor.set(0);
    bottomShooterMotor.set(0);

    // let top motor work up to max speed
    topShooterMotor.set(1);
   // Timer.delay(1.5);

    // feed in next note for shooting
    bottomShooterMotor.set(1);
    Timer.delay(0.5);
    
    // set motors to stop moving
    //topShooterMotor.set(0);
    //bottomShooterMotor.set(0);
  }
}
