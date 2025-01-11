// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LEDs class represents the subsystem responsible for controlling the LEDs on the robot. It
 * provides methods to set colors, patterns, and animations for the LEDs.
 */
public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  private double m_rainbowFirstPixelHue = 0;
  int count = 0;
  boolean toggled = false;

  private Panel leftPanel = new Panel(0, LEDConstants.panelWidth, LEDConstants.panelHeight);
  private Panel rightPanel =
      new Panel(leftPanel.getEnd(), LEDConstants.panelWidth, LEDConstants.panelHeight);

  /** Represents the LEDs subsystem of the robot. */
  public LEDs() {
    // PWM port 2 on the Rio
    m_led = new AddressableLED(0);

    // sets the length of the LEDs
    m_ledBuffer = new AddressableLEDBuffer(getLength());

    setUpLight();

    count = 0;
  }

  /** Sets all LEDs to green color. */
  public void setGreen() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 10, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets up the LED lights by defining the length of the LEDs, setting the LED data, and starting
   * the signal.
   */
  public void setUpLight() {
    // defines the length of the leds
    // setting the length is expensive so only call it once
    m_led.setLength(m_ledBuffer.getLength());

    // sets the LED data and starts the signal
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /** Updates the LED data by setting the LED buffer. */
  public void update() {
    m_led.setData(m_ledBuffer);
  }

  /**
   * Returns the length of the LEDs.
   *
   * @return The length of the LEDs.
   */
  public int getLength() {
    return rightPanel.getEnd();
  }

  /**
   * Returns an array containing the indices of all the LEDs.
   *
   * @return an array of integers representing the indices of all the LEDs
   */
  public int[] getAllLEDs() {
    return new int[] {0, getLength()};
  }

  /**
   * Returns an array representing the range of panels. The first element of the array is the left
   * panel range, and the second element is the right panel range.
   *
   * @return an array representing the range of panels
   */
  public int[] getPanelsRange() {
    return new int[] {getLeftPanelRange()[0], getRightPanelRange()[1]};
  }

  /**
   * Returns the range of the left panel LEDs.
   *
   * @return an array containing the start and end indices of the left panel LEDs
   */
  public int[] getLeftPanelRange() {
    return new int[] {leftPanel.getStart(), leftPanel.getEnd()};
  }

  /**
   * Returns the range of the right panel LEDs.
   *
   * @return an array containing the start and end indices of the right panel LEDs
   */
  public int[] getRightPanelRange() {
    return new int[] {rightPanel.getStart(), rightPanel.getEnd()};
  }

  /**
   * Returns the left panel of the LEDs subsystem.
   *
   * @return the left panel of the LEDs subsystem
   */
  public Panel getLeftPanel() {
    return leftPanel;
  }

  /**
   * Returns the right panel of the LEDs subsystem.
   *
   * @return the right panel of the LEDs subsystem
   */
  public Panel getRightPanel() {
    return rightPanel;
  }

  /**
   * Sets the color of all LEDs to the specified color.
   *
   * @param color an array representing the RGB color values [red, green, blue]
   */
  public void setAllColor(int[] color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets the color of all panels to the specified color.
   *
   * @param color an array representing the RGB color values [red, green, blue]
   */
  public void setAllPanelColor(int[] color) {
    setPanelColor(leftPanel, color);
    setPanelColor(rightPanel, color);
  }

  /**
   * Sets the color of a range of LEDs.
   *
   * @param startEnd an array containing the start and end indices of the LED range
   * @param color an array containing the RGB values of the desired color
   */
  public void setColor(int[] startEnd, int[] color) {
    for (int i = startEnd[0]; i < startEnd[1]; i++) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
  }

  /**
   * Sets the color of a panel.
   *
   * @param panel The panel to set the color for.
   * @param color An array representing the RGB color values.
   */
  public void setPanelColor(Panel panel, int[] color) {
    panel.setAllColor(m_ledBuffer, color);
  }

  /** Turns off all LEDs. */
  public void setAllOff() {
    setAllColor(new int[] {0, 0, 0});
  }

  /**
   * Sets the LEDs to display a rainbow pattern within the specified range.
   *
   * @param startEnd an array containing the start and end indices of the LEDs to be set
   */
  public void setRainbow(int[] startEnd) {
    for (int i = startEnd[0]; i < startEnd[1]; i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }

    for (int i = startEnd[0]; i < startEnd[1]; i++) {
      final int hue = ((int) (m_rainbowFirstPixelHue + (i * 180 / startEnd[1]))) % 180;
      m_ledBuffer.setHSV(i, hue, 150, 200);
    }
    m_rainbowFirstPixelHue += 0.005;

    m_rainbowFirstPixelHue %= 180;
  }

  /**
   * Sets the LED strip to a dim rainbow pattern. Each LED in the strip will have a different hue
   * value, creating a rainbow effect. The brightness and saturation of the LEDs are adjusted to
   * create a dim effect.
   */
  public void dimRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final int hue = ((int) (m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;
      if (i % 2 == 0) {

        m_ledBuffer.setHSV(i, 0, 0, 0);
      } else {
        m_ledBuffer.setHSV(i, hue, 250, 140);
      }

      m_rainbowFirstPixelHue += 0.015;

      m_rainbowFirstPixelHue %= 180;
    }
  }

  public void setAllHSV(int hue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, 200, 50);
    }
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets the shape of the LED panel.
   *
   * @param panel The LED panel to set the shape on.
   * @param shape The shape to set on the LED panel.
   */
  public void setShape(Panel panel, Shape shape) {
    panel.setShape(m_ledBuffer, shape.get());
  }

  /**
   * Mirrors the given 3D array of LED states horizontally.
   *
   * @param state the original 3D array of LED states
   * @return the mirrored 3D array of LED states
   */
  public int[][][] getMirrored(int[][][] state) {
    int[][][] leds = state;
    for (int row = 0; row < leds.length; row++) {
      for (int col = 0; col < leds[0].length; col++) {
        int[] tempColor = leds[row][col];
        leds[row][col] = leds[row][leds[row].length - col - 1];
        leds[row][leds[row].length - col - 1] = tempColor;
      }
    }
    return leds;
  }

  /**
   * Toggles between two panels and updates their shapes and colors based on the given state. If the
   * count is equal to 1, it checks the current toggle state and updates the panels accordingly. If
   * the toggle state is true, it sets the shape of the first panel and sets the color of the second
   * panel. If the toggle state is false, it sets the shape of the second panel and sets the color
   * of the first panel. After updating the panels, it mirrors the state. If the count is greater
   * than 100, it resets the count to 0.
   *
   * @param firstPanel The first panel to toggle.
   * @param secondPanel The second panel to toggle.
   * @param state The state to update the panels with.
   */
  public void togglePanel(Panel firstPanel, Panel secondPanel, int[][][] state) {
    count++;
    if (count == 1) {
      if (toggled) {
        firstPanel.setShape(m_ledBuffer, state);
        setPanelColor(secondPanel, LEDConstants.n);
        toggled = false;
      } else {
        secondPanel.setShape(m_ledBuffer, state);
        setPanelColor(firstPanel, LEDConstants.n);
        toggled = true;
      }
      state = getMirrored(state);
    } else if (count > 100) {
      count = 0;
    }
  }

  /*
   * public void displayGIF(Panel[] panels, Shape[] gif){
   * count++;
   * int fpi = 10;
   * int numFrame = 0;
   * if (numFrame > gif.length){
   * numFrame = 0;
   * }
   * if (count > fpi){
   * count = 0;
   * numFrame++;
   * for (Panel panel : panels){
   * panel.setShape(m_ledBuffer, gif[numFrame].get());
   * }
   * }
   * }
   */

}
