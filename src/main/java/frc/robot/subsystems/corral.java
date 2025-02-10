package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class corral extends SubsystemBase {

  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;
  public double mult = 0.0492;
  private SparkMax thingy = new SparkMax(16, MotorType.kBrushless);

  @Override
  public void periodic() {
      ultrasonicSensorOneRange = ultrasonicSensorOne.getValue() * voltageScaleFactor * mult;
       SmartDashboard.putNumber("Sensor 1 Range", ultrasonicSensorOneRange);
        voltageScaleFactor = 5/RobotController.getVoltage5V();  
  }
  public void intake() {
 if (ultrasonicSensorOneRange >= 17) 
    {
      thingy.set(0.2);
    } else 
    {
      thingy.set(0);
    }
  }
  public void outtake()
  {
        thingy.set(1);
  }
  public void override()
  {
    mult = 0;
  }

  public void renable()
  {
    mult = 0.0492;
  }
  }
