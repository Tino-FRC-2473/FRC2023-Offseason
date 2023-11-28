package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;

public class LED extends TimedRobot{
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    int m_rainbowFirstPixelHue = 0;
    
    public LED() {
        led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(30);
        //ledBuffer = new AddressableLEDBuffer(30); // update LED length no.
		led.setLength(ledBuffer.getLength());

		// RED
		//led.setData(ledBuffer);
		led.start();
    }

    public void rainbow() {
        // For every pixel
        
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
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
