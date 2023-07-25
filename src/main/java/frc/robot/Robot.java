// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.RotatedRect;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;  
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  double numberThatMoves = 0.0;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_timer.start();
    if (m_timer.get() < 2){
      m_robotDrive.arcadeDrive(0.75, 0);
    }
    if (m_timer.get() > 2 && m_timer.get() < 2.3)
      m_robotDrive.arcadeDrive(0, 0.75);
  }

  /** Thisunction is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");
    
    inst.startClient4("85"); // Make sure you set this to your team number
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
    
    // NetworkTableEntry TeamEntry = table.getEntry("tx");
    NetworkTableEntry xEntry = table.getEntry("tx");
    NetworkTableEntry yEntry = table.getEntry("ty");
    NetworkTableEntry aEntry = table.getEntry("ta");
    NetworkTableEntry lEntry = table.getEntry("tl");
    NetworkTableEntry vEntry = table.getEntry("tv");
    NetworkTableEntry sEntry = table.getEntry("ts");
    NetworkTableEntry aprilEntry = table.getEntry("tid");
    
    NetworkTableEntry tshortEntry = table.getEntry("tshort");
    NetworkTableEntry tlongEntry = table.getEntry("tlong");
    NetworkTableEntry thorEntry = table.getEntry("thor");
    NetworkTableEntry tvertEntry = table.getEntry("tvert");
    NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
    NetworkTableEntry camtranEntry = table.getEntry("camtran");
    NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
    
    // double tx = xEntry.getDouble(0.0);
    double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                      // latency.
    double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
    double tid = aprilEntry.getDouble(0.0); // April Tag Number
    
    // double tshort = tshortEntry.getString(); // Sidelength of shortest side of
    // the fitted bounding box (pixels)
    // double tlong = tlong // Sidelength of longest side of the fitted bounding box
    // (pixels)
    // double thor = thor // Horizontal sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double tvert = tvert // Vertical sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double getpipe = getpipe // True active pipeline index of the camera (0 .. 9)
    // double camtran = camtran // Results of a 3D position solution, 6 numbers:
    // Translation (x,y,y) Rotation(pitch,yaw,roll)
    
    //ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    ledModeEntry.setNumber(1); // force off
    //ledModeEntry.setNumber(2); // force blink
    //ledModeEntry.setNumber(3); // force on
    
    //System.out.println("X: " + tx);
    //System.out.println("Y: " + ty);
    //System.out.println("A: " + ta);
    //System.out.println("L: " + tl);
    //System.out.println("V: " + tv);
    //System.out.println("S: " + tv);
    
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putNumber("Limelight Skew", ts);
    SmartDashboard.putNumber("April Tag ID", tid);
    
    // Limelight Data End
    if (m_stick.getRawAxis(2) < 0.5){
      m_robotDrive.arcadeDrive(m_stick.getRawAxis(3), m_stick.getX()*-1);
    } else {
      m_robotDrive.arcadeDrive(-m_stick.getRawAxis(2), m_stick.getX()*-1);
    
    }
    
    //Shuffleboard YStick = new Shuffleboard.getTab("YStick");
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
