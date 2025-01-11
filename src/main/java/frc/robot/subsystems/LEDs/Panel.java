// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Panel extends SubsystemBase {
  /** Creates a new Panel. */
  private int width;

  private int height;
  private final int start;
  private final int end;
  private int[][][] leds;

  /**
   * Represents a panel of LEDs. Each panel has a specified width and height, and contains an array
   * of RGB values for each LED. The panel also keeps track of its start and end indices.
   *
   * @param start the start index of the panel
   * @param width the width of the panel
   * @param height the height of the panel
   */
  public Panel(int start, int width, int height) {
    this.width = width;
    this.height = height;
    leds = new int[width][height][3];
    this.start = start;
    this.end = start + getLength();
  }

  /**
   * Returns the length of the panel, calculated by multiplying the width and height.
   *
   * @return the length of the panel
   */
  public int getLength() {
    return width * height;
  }

  /**
   * Returns the start value.
   *
   * @return the start value
   */
  public int getStart() {
    return start;
  }

  /**
   * Returns the end value.
   *
   * @return the end value
   */
  public int getEnd() {
    return end;
  }

  /**
   * Returns the current state of the LEDs.
   *
   * @return a 3-dimensional array representing the LEDs
   */
  public int[][][] getLEDs() {
    return leds;
  }

  /**
   * Calculates the index of an LED based on the given row and column.
   *
   * @param row the row of the LED
   * @param col the column of the LED
   * @return the index of the LED
   */
  public int getIndex(int row, int col) {
    int index = start;
    index += (row * 16) + col;
    return index;
  }

  /**
   * Writes the LED values from the internal 3D array to the provided AddressableLEDBuffer.
   *
   * @param m_ledBuffer the AddressableLEDBuffer to write the LED values to
   * @return the updated AddressableLEDBuffer with the LED values written
   */
  public AddressableLEDBuffer writeLEDs(AddressableLEDBuffer m_ledBuffer) {
    for (int row = 0; row < leds.length; row++) {
      for (int col = 0; col < leds[0].length; col++) {
        m_ledBuffer.setRGB(
            getIndex(row, col), leds[row][col][0], leds[row][col][1], leds[row][col][2]);
      }
    }
    return m_ledBuffer;
  }

  /**
   * Represents a buffer of addressable LEDs.
   *
   * @param m_ledBuffer the AddressableLEDBuffer to set the color on
   * @param color the RGB color values to set on all LEDs in the buffer
   * @return the updated AddressableLEDBuffer with the color set on all LEDs
   */
  public AddressableLEDBuffer setAllColor(AddressableLEDBuffer m_ledBuffer, int[] color) {
    for (int i = start; i < end; i++) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
    return m_ledBuffer;
  }

  /**
   * Sets the shape of the LEDs in the AddressableLEDBuffer.
   *
   * @param m_ledBuffer The AddressableLEDBuffer to set the shape on.
   * @param shape The shape of the LEDs represented as a 3D array. The first dimension represents
   *     the rows, the second dimension represents the columns, and the third dimension represents
   *     the LED values.
   * @return The updated AddressableLEDBuffer after setting the shape.
   */
  public AddressableLEDBuffer setShape(AddressableLEDBuffer m_ledBuffer, int[][][] shape) {
    for (int i = 0; i < shape.length; i++) {
      for (int j = 0; j < shape[0].length; j++) {
        leds[i][j] = shape[i][j];
      }
    }
    return writeLEDs(m_ledBuffer);
  }

  /**
   * Represents a buffer of addressable LEDs.
   *
   * @param m_ledBuffer the AddressableLEDBuffer to mirror the LEDs on
   * @param mirror the 3D array representing the mirrored LED values
   * @return the updated AddressableLEDBuffer with the LEDs mirrored
   */
  public AddressableLEDBuffer mirror(AddressableLEDBuffer m_ledBuffer, int[][][] mirror) {
    leds = mirror;
    for (int row = 0; row < leds.length; row++) {
      for (int col = 0; col < leds[0].length / 2; col++) {
        int[] tempColor = leds[row][col];
        leds[row][col] = leds[row][leds[row].length - col - 1];
        leds[row][leds[row].length - col - 1] = tempColor;
      }
    }
    return writeLEDs(m_ledBuffer);
  }
}
