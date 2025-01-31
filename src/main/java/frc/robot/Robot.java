// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

public class Robot extends TimedRobot {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private Command m_autonomousCommand;
  private Joystick m_stick;
  private Joystick stick;
  public static String color = "need update";
   public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  private final RobotContainer m_robotContainer;
  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;
  public double mult = 0.0492;
  private SparkMax thingy = new SparkMax(16, MotorType.kBrushless);
      Compressor c = new Compressor(1, PneumaticsModuleType.CTREPCM);
  PneumaticsControlModule pH = new PneumaticsControlModule(1); 

  DoubleSolenoid shooter = pH.makeDoubleSolenoid(1,2);
  @SuppressWarnings("unused")
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  LEDPattern red = LEDPattern.solid(Color.kGreen);
  LEDPattern blue =  LEDPattern.solid(Color.kBlue);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */ public void robotInit() {

     m_stick = new Joystick(1);
     stick = new Joystick(0);}
    //Initialize range readings on SmartDashboard as max distance in Centimeters.
  public Robot() {
    m_led = new AddressableLED(8);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(300);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    @SuppressWarnings("unused")
    Distance ledSpacing = Meters.of(1 / 120.0);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
 
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  // Our LED strip has a density of 120 LEDs per meter

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @SuppressWarnings("unlikely-arg-type")
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Sensor 1 Range", ultrasonicSensorOneRange);
        voltageScaleFactor = 5/RobotController.getVoltage5V();    // Update the buffer with the rainbow animation
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
          blue.applyTo(m_ledBuffer);
          m_led.setData(m_ledBuffer);
        } else {
          red.applyTo(m_ledBuffer);
          m_led.setData(m_ledBuffer);
        }
  }
  public void setLEDColor(String color) {
    SmartDashboard.putString("LED Color", color); // Send color to dashboard
    // Optionally, add code here to physically change the LED color on your robot
}
  /** This function is called once each time the robot enters Disabled mode. */
   @Override
  public void disabledInit() {
  
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() { 
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {  
    if (m_stick.getRawButton(3) & ultrasonicSensorOneRange >= 17) { 
      thingy.set(.2);
    } else { 
      thingy.set(0);
    }
    ultrasonicSensorOneRange = ultrasonicSensorOne.getValue() * voltageScaleFactor * mult;
    
    if (m_stick.getRawButton(4)) { 
      thingy.set(1);
    }
    
    if (m_stick.getRawButton(11)) { 
      mult = 0;
    }
    
    if (m_stick.getRawButton(12)) { 
      mult = 0.0492;
    }
    if (stick.getRawButton(1))
    {
      shooter.set(Value.kForward);
    
    }
    if (stick.getRawButton(2)) {
      shooter.set(Value.kReverse);
    }
  }
  


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
