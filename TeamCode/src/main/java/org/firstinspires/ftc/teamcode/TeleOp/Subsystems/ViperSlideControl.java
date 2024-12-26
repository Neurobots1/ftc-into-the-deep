package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ViperSlideControl {
    private PIDController controller;

    // PID constants
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;

    // Target position
    private static int target = 0;

    // Tolerance for stopping (if the motor is within this range, we consider it done)
    private static final int TOLERANCE = 50;

    // Motors
    private DcMotorEx slidemotorLeft;
    private DcMotorEx slidemotorRight;

    // Constructor
    public ViperSlideControl(DcMotorEx slidemotorLeft, DcMotorEx slidemotorRight) {
        this.slidemotorLeft = slidemotorLeft;
        this.slidemotorRight = slidemotorRight;
        controller = new PIDController(p, i, d);

        // Set initial motor configurations
        slidemotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidemotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidemotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to update the Viper Slide position
    public void update() {
        // Only apply PID if the target position is not already reached
        if (!isAtTarget()) {
            // Get current positions of both slide motors
            int slidePosLeft = slidemotorLeft.getCurrentPosition();
            int slidePosRight = slidemotorRight.getCurrentPosition();

            // Calculate PID output for both motors
            double pidLeft = controller.calculate(slidePosLeft, target);
            double pidRight = controller.calculate(slidePosRight, target);

            // Feedforward calculation (this is optional and can be adjusted)
            double ff = Math.cos(Math.toRadians(target / 537.7)) * f;

            // Compute motor power
            double powerLeft = pidLeft + ff;
            double powerRight = pidRight + ff;

            // Set motor power
            slidemotorLeft.setPower(powerLeft);
            slidemotorRight.setPower(powerRight);
        } else {
            // If the target is reached, stop the motors
            slidemotorLeft.setPower(0);
            slidemotorRight.setPower(0);
        }
    }

    // Method to set the target position
    public void setTarget(int newTarget) {
        target = newTarget;
        // Reset PID controller to start fresh
        controller.reset();
    }

    // Method to check if the target position is reached
    private boolean isAtTarget() {
        int slidePosLeft = slidemotorLeft.getCurrentPosition();
        int slidePosRight = slidemotorRight.getCurrentPosition();
        return Math.abs(slidePosLeft - target) < TOLERANCE && Math.abs(slidePosRight - target) < TOLERANCE;
    }

    // Getter methods for motor positions and target
    public int getLeftPosition() {
        return slidemotorLeft.getCurrentPosition();
    }

    public int getRightPosition() {
        return slidemotorRight.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }
}
