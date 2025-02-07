package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class climber extends SubsystemBase {
    Compressor c = new Compressor(1, PneumaticsModuleType.CTREPCM); 
      PneumaticsControlModule pH = new PneumaticsControlModule(1); 
 
      DoubleSolenoid shooter = pH.makeDoubleSolenoid(0,1);


  /** Grabs the hatch. */
  public void climb() {
    shooter.set(kReverse);
  }

  /** Releases the hatch. */
  public void release() {
    shooter.set(kForward);
  }


 
    
  }

