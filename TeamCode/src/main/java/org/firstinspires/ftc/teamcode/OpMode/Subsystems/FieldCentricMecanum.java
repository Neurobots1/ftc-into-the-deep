package org.firstinspires.ftc.teamcode.OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricMecanum {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private IMU imu;
    private double headingSetpoint = 0.0;
    private final double headingKp;

    public FieldCentricMecanum(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, IMU imu, double headingKp) {
        this.leftFront = leftFront;
        this.leftRear = leftBack;
        this.rightFront = rightFront;
        this.rightRear = rightBack;
        this.imu = imu;
        this.headingKp = headingKp;
    }

    public void drive(Gamepad gamepad) {
        // Mecanum Drive Control with Heading Lock
        double y = -gamepad.left_stick_y; // Y-axis (reversed)
        double x = gamepad.left_stick_x;  // X-axis (strafing)
        double rx = gamepad.right_stick_x; // Rotation

        // Reset IMU yaw if the options button is pressed
        if (gamepad.options) {
            imu.resetYaw();
        }

        // Get current robot heading (yaw) from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Update the heading setpoint when there is manual rotation input
        if (Math.abs(rx) > 0.05) {
            headingSetpoint = botHeading;
        }

        // Calculate the heading error and apply proportional control
        double headingError = headingSetpoint - botHeading;
        double headingCorrection = headingKp * headingError;

        // Apply heading correction when there is no manual rotation input
        if (Math.abs(rx) < 0.05) {
            rx = headingCorrection;
        }

        // Calculate rotated movement direction (field-centric driving)
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing (adjust for motor imperfections)

        // Normalize motor power to ensure all motor values are within [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers for Mecanum drive
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}

