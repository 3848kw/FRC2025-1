package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class corral extends SubsystemBase {

  public AnalogInput ultrasonicSensorOne = new AnalogInput(3);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;
  public double mult = 0.0492;
    DigitalInput limitSwitch = new DigitalInput(0);
    DigitalInput Switch = new DigitalInput(1);
    Joystick exampleJoystick = new Joystick(1); // 0 is the USB Port to be used as indicated on the Driver Station

  private SparkMax thingy = new SparkMax(16, MotorType.kBrushless);
  public double stop = 17;
  @Override
  public void periodic() {
      ultrasonicSensorOneRange = ultrasonicSensorOne.getValue() * voltageScaleFactor * mult;
       SmartDashboard.putNumber("Sensor 1 Range", ultrasonicSensorOneRange);
        voltageScaleFactor = 5/RobotController.getVoltage5V();  

  }
  public void intake() {
   
      
      
     
    if (ultrasonicSensorOneRange > stop)
    {
      thingy.set(0.2);
    }
    else  {
      thingy.set(0);
    }
  }
  
  
  public void outtake()
  {
  thingy.set(1);
  }

  public void renable()
  {
  
  }
  }
