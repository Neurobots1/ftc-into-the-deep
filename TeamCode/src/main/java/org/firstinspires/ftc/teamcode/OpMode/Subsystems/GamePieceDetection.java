package org.firstinspires.ftc.teamcode.OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.LinkedList;
import java.util.Queue;

public class GamePieceDetection {

    private ColorSensor colorSensor;

    // A queue to store the last 50 readings for averaging
    private static final int MAX_READINGS = 50;
    private Queue<Integer> redQueue = new LinkedList<>();
    private Queue<Integer> greenQueue = new LinkedList<>();
    private Queue<Integer> blueQueue = new LinkedList<>();

    private double averageRed = 0, averageGreen = 0, averageBlue = 0;

    // Proportions for when the game piece is closest
    private final double RED_PROPORTION_R_CLOSE = 0.561, RED_PROPORTION_G_CLOSE = 0.296, RED_PROPORTION_B_CLOSE = 0.143;
    private final double BLUE_PROPORTION_R_CLOSE = 0.144, BLUE_PROPORTION_G_CLOSE = 0.284, BLUE_PROPORTION_B_CLOSE = 0.571;
    private final double YELLOW_PROPORTION_R_CLOSE = 0.392, YELLOW_PROPORTION_G_CLOSE = 0.497, YELLOW_PROPORTION_B_CLOSE = 0.111;
    private final double NO_GAME_PROPORTION_R_CLOSE = 0.244, NO_GAME_PROPORTION_G_CLOSE = 0.423, NO_GAME_PROPORTION_B_CLOSE = 0.333;

    // Proportions for when the game piece is farthest
    private final double RED_PROPORTION_R_FAR = 0.370, RED_PROPORTION_G_FAR = 0.430, RED_PROPORTION_B_FAR = 0.200;
    private final double BLUE_PROPORTION_R_FAR = 0.060, BLUE_PROPORTION_G_FAR = 0.170, BLUE_PROPORTION_B_FAR = 0.770;
    private final double YELLOW_PROPORTION_R_FAR = 0.280, YELLOW_PROPORTION_G_FAR = 0.500, YELLOW_PROPORTION_B_FAR = 0.220;
    private final double NO_GAME_PROPORTION_R_FAR = 0.200, NO_GAME_PROPORTION_G_FAR = 0.460, NO_GAME_PROPORTION_B_FAR = 0.340;

    // Proportional matching threshold (tweak as needed)
    private final double THRESHOLD = 0.04; // Allowable difference in proportions

    // Variable to store the detected color
    private String detectedColor = "None"; // Default to "None" initially

    // Constructor
    public GamePieceDetection(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    // Update the rolling averages for RGB values
    private void updateRollingAverages(int red, int green, int blue) {
        // Add new values to the queues
        redQueue.add(red);
        greenQueue.add(green);
        blueQueue.add(blue);

        // Remove oldest values if the queue exceeds the max size
        if (redQueue.size() > MAX_READINGS) redQueue.poll();
        if (greenQueue.size() > MAX_READINGS) greenQueue.poll();
        if (blueQueue.size() > MAX_READINGS) blueQueue.poll();

        // Calculate averages
        averageRed = redQueue.stream().mapToInt(Integer::intValue).average().orElse(0);
        averageGreen = greenQueue.stream().mapToInt(Integer::intValue).average().orElse(0);
        averageBlue = blueQueue.stream().mapToInt(Integer::intValue).average().orElse(0);
    }

    // Determine the color of the game piece based on RGB proportions
    public void detectColor() {
        // Get RGB values from the sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Update the rolling averages
        updateRollingAverages(red, green, blue);

        // Normalize the values by dividing each by the total intensity (R + G + B)
        double totalIntensity = averageRed + averageGreen + averageBlue;

        if (totalIntensity == 0) {
            detectedColor = "None"; // No color detected if all channels are 0
            return;
        }

        // Normalize each channel to its proportion of the total intensity
        double normRed = averageRed / totalIntensity;
        double normGreen = averageGreen / totalIntensity;
        double normBlue = averageBlue / totalIntensity;

        // Try to match the closest proportions first
        if (isProportionalMatch(normRed, normGreen, normBlue, RED_PROPORTION_R_CLOSE, RED_PROPORTION_G_CLOSE, RED_PROPORTION_B_CLOSE)) {
            detectedColor = "Red"; // Red game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, BLUE_PROPORTION_R_CLOSE, BLUE_PROPORTION_G_CLOSE, BLUE_PROPORTION_B_CLOSE)) {
            detectedColor = "Blue"; // Blue game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, YELLOW_PROPORTION_R_CLOSE, YELLOW_PROPORTION_G_CLOSE, YELLOW_PROPORTION_B_CLOSE)) {
            detectedColor = "Yellow"; // Yellow game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, NO_GAME_PROPORTION_R_CLOSE, NO_GAME_PROPORTION_G_CLOSE, NO_GAME_PROPORTION_B_CLOSE)) {
            detectedColor = "None"; // No game piece detected
        } else {
            // If no match found for the closest proportions, check for the farthest proportions
            if (isProportionalMatch(normRed, normGreen, normBlue, RED_PROPORTION_R_FAR, RED_PROPORTION_G_FAR, RED_PROPORTION_B_FAR)) {
                detectedColor = "Red"; // Red game piece
            } else if (isProportionalMatch(normRed, normGreen, normBlue, BLUE_PROPORTION_R_FAR, BLUE_PROPORTION_G_FAR, BLUE_PROPORTION_B_FAR)) {
                detectedColor = "Blue"; // Blue game piece
            } else if (isProportionalMatch(normRed, normGreen, normBlue, YELLOW_PROPORTION_R_FAR, YELLOW_PROPORTION_G_FAR, YELLOW_PROPORTION_B_FAR)) {
                detectedColor = "Yellow"; // Yellow game piece
            } else if (isProportionalMatch(normRed, normGreen, normBlue, NO_GAME_PROPORTION_R_FAR, NO_GAME_PROPORTION_G_FAR, NO_GAME_PROPORTION_B_FAR)) {
                detectedColor = "None"; // No game piece detected
            } else {
                detectedColor = "Unknown"; // Unknown or not matching any predefined color
            }
        }
    }

    // Get the detected color
    public String getDetectedColor() {
        return detectedColor;
    }

    // Check if the proportions are close to the predefined ratios
    private boolean isProportionalMatch(double red, double green, double blue, double targetRed, double targetGreen, double targetBlue) {
        // Check if the proportions are close to the predefined ratios
        return Math.abs(red - targetRed) < THRESHOLD &&
                Math.abs(green - targetGreen) < THRESHOLD &&
                Math.abs(blue - targetBlue) < THRESHOLD;
    }
}
