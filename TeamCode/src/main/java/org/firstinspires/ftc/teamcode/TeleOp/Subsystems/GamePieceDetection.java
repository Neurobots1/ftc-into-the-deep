package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

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

    // Predefined normalized RGB proportions for each game piece
    private final double NO_GAME_PROPORTION_R = 0.2316, NO_GAME_PROPORTION_G = 0.4211, NO_GAME_PROPORTION_B = 0.3474;
    private final double BLUE_PROPORTION_R = 0.1363, BLUE_PROPORTION_G = 0.2733, BLUE_PROPORTION_B = 0.5904;
    private final double YELLOW_PROPORTION_R = 0.3935, YELLOW_PROPORTION_G = 0.5056, YELLOW_PROPORTION_B = 0.1010;
    private final double RED_PROPORTION_R = 0.5747, RED_PROPORTION_G = 0.2895, RED_PROPORTION_B = 0.1375;

    // Proportional matching threshold (tweak as needed)
    private final double THRESHOLD = 0.05; // Allowable difference in proportions

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

        // Compare the normalized values to the predefined ratios for each game piece
        if (isProportionalMatch(normRed, normGreen, normBlue, RED_PROPORTION_R, RED_PROPORTION_G, RED_PROPORTION_B)) {
            detectedColor = "Red"; // Red game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, BLUE_PROPORTION_R, BLUE_PROPORTION_G, BLUE_PROPORTION_B)) {
            detectedColor = "Blue"; // Blue game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, YELLOW_PROPORTION_R, YELLOW_PROPORTION_G, YELLOW_PROPORTION_B)) {
            detectedColor = "Yellow"; // Yellow game piece
        } else if (isProportionalMatch(normRed, normGreen, normBlue, NO_GAME_PROPORTION_R, NO_GAME_PROPORTION_G, NO_GAME_PROPORTION_B)) {
            detectedColor = "None"; // No game piece detected
        } else {
            detectedColor = "Unknown"; // Unknown or not matching any predefined color
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
