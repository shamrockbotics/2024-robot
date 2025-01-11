// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Strip extends SubsystemBase {
  /** Creates a new Strip. */
  private int length;

  private final int start;
  private final int end;

  private int[][] leds;

  /**
   * Constructs a new Strip object with the specified start index and length.
   *
   * @param start The starting index of the LED strip.
   * @param length The length of the LED strip.
   */
  public Strip(int start, int length) {
    this.length = length;
    leds = new int[length][3];
    this.start = start;
    this.end = start + getLength();
  }

  /**
   * Returns the length of the LED strip.
   *
   * @return The length of the LED strip.
   */
  public int getLength() {
    return length;
  }

  /**
   * Returns the starting index of the LED strip.
   *
   * @return The starting index of the LED strip.
   */
  public int getStart() {
    return start;
  }

  /**
   * Returns the end value of the LED strip.
   *
   * @return The end value of the LED strip.
   */
  public int getEnd() {
    return end;
  }

  /**
   * Returns the 2D array representing the LEDs.
   *
   * @return The 2D array of LEDs.
   */
  public int[][] getLEDs() {
    return leds;
  }

  /**
   * Returns the adjusted index based on the starting index.
   *
   * @param index The original index.
   * @return The adjusted index.
   */
  public int getIndex(int index) {
    return index + start;
  }

  /**
   * Sets the color of all LEDs in the strip to the specified color.
   *
   * @param m_ledBuffer The addressable LED buffer.
   * @param color The color to set (an array of RGB values).
   * @return The updated addressable LED buffer.
   */
  public AddressableLEDBuffer setAllColor(AddressableLEDBuffer m_ledBuffer, int[] color) {
    for (int i = 0; i < getLength(); i++) {
      leds[i] = color;
      m_ledBuffer.setRGB(getIndex(i), color[0], color[1], color[2]);
    }
    return m_ledBuffer;
  }
}
