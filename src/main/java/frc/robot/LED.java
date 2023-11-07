package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;

public class LED extends TimedRobot{
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    @Override
    public void robotInit() {
      led = new AddressableLED(0); // update PWM port no.
  
      ledBuffer = new AddressableLEDBuffer(60); // update LED length no.
      led.setLength(ledBuffer.getLength());

      // RED
      setRGBVals(255, 0, 0);
      setHSVVals(0, 100, 100);
      led.setData(ledBuffer);
      led.start();
    }

    public void setRGBVals(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, r, g, b);
         }
         led.setData(ledBuffer);
    }

    public void setHSVVals(int hue, int saturation, int val) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            ledBuffer.setHSV(i, hue, saturation, val);
         }
         
         led.setData(ledBuffer);
    }
    
}
