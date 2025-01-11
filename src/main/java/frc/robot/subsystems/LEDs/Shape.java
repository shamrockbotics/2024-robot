// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shape extends SubsystemBase {
  /**
   * Creates a new Shape.
   *
   * @param initShape the initial shape as a 3D integer array
   */
  private int[][][] image;

  public Shape(int[][][] initShape) {
    image = setUpShape(initShape);
  }

  /**
   * Returns the image data of the shape.
   *
   * @return the image data as a 3D integer array
   */
  public int[][][] get() {
    return image;
  }

  /**
   * Reverses the 15th row of the given shape, rotates the image 90 degrees, and returns the
   * modified shape.
   *
   * @param shape the 3D array representing the shape
   * @return the modified shape with the reversed 15th row and rotated image
   */
  public static int[][][] setUpShape(int[][][] shape) {
    // need to reverse the 15th row
    int[][] fifteen = shape[14];
    for (int i = 0; i < fifteen.length / 2; i++) {
      int[] temp = fifteen[i];
      fifteen[i] = fifteen[fifteen.length - i - 1];
      fifteen[fifteen.length - 1 - i] = temp;
    }
    shape[14] = fifteen;

    // reverse the rows then rotate the image 90 degrees.
    shape = rotateShape(shape);

    return shape;
  }

  /**
   * Rotates a given shape by 90 degrees clockwise.
   *
   * @param shape The shape to be rotated.
   * @return The rotated shape.
   */
  public static int[][][] rotateShape(int[][][] shape) {
    int numRows = shape.length;
    int numCols = shape[0].length;
    // create a new 3d array to store the rotated values
    int[][][] rotatedShape = new int[numRows][numCols][shape[0][0].length];
    // rotate the shape by 90 degrees clockwise

    for (int i = 0; i < numCols; i++) {
      for (int j = 0; j < numRows; j++) {
        rotatedShape[i][j] = shape[numRows - j - 1][i];
      }
    }

    // return the rotated shape
    return rotatedShape;
  }
}
