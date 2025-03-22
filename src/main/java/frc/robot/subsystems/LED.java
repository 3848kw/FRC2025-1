package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

public LED() {
    
m_led = new AddressableLED(8);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(300);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

}

public void blue()
{
    
    LEDPattern blue =  LEDPattern.solid(Color.kBlue);
    blue.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
}
public void red()
{

    LEDPattern red =  LEDPattern.solid(Color.kGreen);
    red.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
}
public void shit()
{
    m_ledBuffer.setRGB(1, 255, 192, 203);
    m_led.setData(m_ledBuffer);
}
}
