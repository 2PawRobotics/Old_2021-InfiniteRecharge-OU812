/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.kauailabs.navx.frc.AHRS;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive tank;

  private Joystick rightjoy;
  private Joystick leftjoy;
  private Joystick operator;
  private Timer timer = new Timer();

  private final Spark lift = new Spark(6);
  private final Spark balleater = new Spark(5);
  private final Spark shoot = new Spark(4);
  private final Spark spinnything = new Spark(7);
  private final Spark liftlock = new Spark(8);
  private AHRS gyro;
  private double DEAD_ZONE = 0.05; //dead zone for controlers
  private boolean tankDriveMode = false; 
  private double targetangle = 0;
  private double gyrobias = 0.05;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private double currentgyroangle = 0;
  private double gyrocorrectionvalue = 0;
  private double ANGLE_DEAD_ZONE = 2;
  private double NGANGLE_DEAD_ZONE = -2;
  private double Yaccel;
  private double ydistancetraveled = 0;
  private double newydistancetraveled = 0;
  private double distancebias = 0.05;
  private double finalyaccel;
  private boolean moving;
  public Robot()
  {
    System.out.println("Robot.constructor()");
  }
  @Override
  public void robotInit() {
    System.out.println("Robot.robotInit()");

   final SpeedController left = new SpeedControllerGroup(new Spark(0), new Spark(2));
   final SpeedController right = new SpeedControllerGroup(new Spark(1), new Spark(3));

    tank = new DifferentialDrive(left, right);
    rightjoy = new Joystick(0);
    leftjoy = new Joystick(1);
    operator = new Joystick(2);
    CameraServer.getInstance().startAutomaticCapture();
    gyro = new AHRS(i2cPort);
    gyro.reset();
    gyro.calibrate();
  }

  public double getLeftJoy(){
    double raw = leftjoy.getY();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : raw; //if true returns 0.0 else return raw
  }
  public double getRightJoy(){
    double raw = rightjoy.getY();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : raw; //if true returns 0.0 else return raw
  }
  public double getRightJoyX(){
    double raw = rightjoy.getX();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : -raw; //if true returns 0.0 else return raw
  }
  private void gyrodrivecorrection(){
     
    gyrocorrectionvalue = targetangle - currentgyroangle;
  }
  private void distancemeasuring(){ //this method measures distance
    
    if (Math.abs(Yaccel) < distancebias){
      finalyaccel = 0.0; }
    else{
      finalyaccel = Yaccel;
    }
    newydistancetraveled = ydistancetraveled + finalyaccel;
    ydistancetraveled = newydistancetraveled;
  }
  @Override
  public void teleopPeriodic() { //1 is 100% so .2 is 20%

    if(rightjoy.getRawButton(5)){tankDriveMode = true;}
    if(rightjoy.getRawButton(6)){tankDriveMode = false;}

    if(tankDriveMode){
      tank.tankDrive(getLeftJoy(), getRightJoy(), true);
    }
    else{
      tank.arcadeDrive(getRightJoy(), getRightJoyX(), true);
    }
    distancemeasuring();
    System.out.println("distance traveled" + ydistancetraveled);

    System.out.println("gyro angle is" + gyro.getAngle());
    System.out.println("Distance is " + gyro.getWorldLinearAccelY());
  if (operator.getRawButton(12)) {
    lift.set(-1);
    } else if (operator.getRawButton(11)) {
        lift.set(1);
    } else {
        lift.set(0);
    }

  if (operator.getRawButton(3)) {
  balleater.set(-0.7);
  } else if (operator.getRawButton(4)) {
  balleater.set(0.4);
  } else {
  balleater.set(0);
  }

  if (operator.getRawButton(2)) { //this is the statment for shooting forward
  shoot.set(-1);
} else if (operator.getRawButton(1)) { //this is a statment for reversing the shooter
  shoot.set(1);
} else {
  shoot.set(0);
}

  if (operator.getRawButton(7)) {
        spinnything.set(1);
      }
      else {
        spinnything.set(0);
  }
  
  if (operator.getRawButton(9)) {
  liftlock.set(-1);
} else if (operator.getRawButton(10)) {
  liftlock.set(1);
} else {
  liftlock.set(0);
}


  }


  @Override
  public void autonomousInit()
   {
    System.out.println("Robot.autonomous()");
    timer.reset();
    timer.start();

    tank.setSafetyEnabled(false);
    gyro.reset();
    gyro.calibrate();
    ydistancetraveled = 0;
   }
  public void ZeroTurn(){
    if (gyrocorrectionvalue > ANGLE_DEAD_ZONE){
       tank.tankDrive(-0.275, 0.275, false);
       
      }
      else if(gyrocorrectionvalue < NGANGLE_DEAD_ZONE){
       tank.tankDrive(0.275, -0.275, false);
       
      }
    }
    public void MovingTurn(){
      if (gyrocorrectionvalue > ANGLE_DEAD_ZONE){
         tank.tankDrive(-0.5, -0.4, false);
         
        }
        else if(gyrocorrectionvalue < NGANGLE_DEAD_ZONE){
         tank.tankDrive(-0.4, -0.5, false);
         
        }
      }
   @Override
   public void autonomousPeriodic() { //autonomous code
    currentgyroangle = gyro.getAngle();
    Yaccel = gyro.getRawAccelY();
    gyrodrivecorrection();
   distancemeasuring();
   System.out.println("gyro correction value is " + gyrocorrectionvalue);
   System.out.println("distance traveled" + ydistancetraveled);
   System.out.println("Yaccel = " + Yaccel);
   
   if (ydistancetraveled <= 0 && ydistancetraveled >= -1){
    targetangle = 0;
   System.out.println("this is true"); }
  else {
    System.out.println("disable");
    
  }


  
      while (Math.abs(gyrocorrectionvalue) > 15){
        ZeroTurn();
      }
      while (Math.abs(gyrocorrectionvalue) > ANGLE_DEAD_ZONE){
        MovingTurn();
      }
      tank.tankDrive(-0.4, -0.4, false);
      System.out.println("3");
     
      
     

   }

  public void operatorControl ()
  {
    System.out.println("Robot.operatorControl()");

    tank.setSafetyEnabled(true);
  }

  public void disable()
  {
    System.out.println("Robot.disable()");
  }
  public void test()
  {
    System.out.println("Robot.test");
  }
}