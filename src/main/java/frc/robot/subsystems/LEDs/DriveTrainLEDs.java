package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainLEDs extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;

    static int hue = 80;
    static int value = 255;

    /** Creates a new LEDs. */
    public DriveTrainLEDs() {
        m_led = new AddressableLED(1);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(52);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        DriveTrainLEDs.hue = 80;
    }

    public static int getHue(){ return hue; }

    public static void setHue(int hue){
        DriveTrainLEDs.hue = hue;
    }

    public static void setHueLerp(double[] range, double lerpVal){
        DriveTrainLEDs.hue = (int) Lerp(range, lerpVal) % 181;
    }

    public static void setValue(int value){
        DriveTrainLEDs.value = value;
    }

    private static double Lerp(double[] range, double lerpVal){
        return (range[1]-range[0]) * lerpVal + range[0];
    }

    @Override
    public void periodic() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hue, 255, value);
        }
        m_led.setData(m_ledBuffer);
    }
}