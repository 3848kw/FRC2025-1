package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class corral extends SubsystemBase {
    
    private SparkMax thingy = new SparkMax(16, MotorType.kBrushless);  // Motor (thingy)
    DigitalInput toplimitSwitch = new DigitalInput(0);
        
  
    @Override
    public void periodic() {
        // Update the ultrasonic sensor's distance
      SmartDashboard.putBoolean("BOB?", toplimitSwitch.get());
        
        // Rounded distance for display purposes


    }
  
    // Intake function that checks for obstruction before running the motor
    public void intake() {
        // If the sensor detects something too close OR the sensor is out of the acceptable range, stop the motor
        if (toplimitSwitch.get()) {
            thingy.set(0);  // Stop the motor
        } else {
            thingy.set(.5);  // Run motor at 0.2 speed if no obstruction and within range
        }
    }
  
    // Outtake function (motor runs at full speed)
    public void outtake() {
        thingy.set(-.2);  // Run the motor at full speed
    }
    public void l2() {
        thingy.set(-.1);  // Run the motor at full speed
    }
  
    // Stop the motor immediately
    public void stop() {
        thingy.set(0);
    }
  
    // Command for intake (can be called by a button press or other event)
    public Command in() {
        return this.run(() -> intake());  // Run the intake method with obstruction checking
    }
  
    // Command for outtake (can be called by a button press or other event)
    public Command out() {
        return this.run(() -> outtake());  // Full speed outtake
    }
      public Command spitCoralOut(double speed)
  {
    return run(() -> {
      thingy.set(-.3);
    });
  }
  
  }
  
