package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class test_intake extends OpMode {

    // Declare the intake motor
    private DcMotor intakeMotor;

    @Override
    public void init() {
        // Initialize the intake motor from the hardware map
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");

        // Set the motor's zero power behavior to BRAKE for better control
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Optionally, set the motor's direction if needed
        // Uncomment the line below if the motor spins in the wrong direction
        // intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Check if Y button is pressed to run the intake motor forward
        if (gamepad1.y) {
            intakeMotor.setPower(0.3); // Set motor to full speed forward
        }
        // Check if A button is pressed to run the intake motor in reverse
        else if (gamepad1.a) {
            intakeMotor.setPower(-1.0); // Set motor to full speed reverse
        }
        // If neither button is pressed, stop the motor
        else {
            intakeMotor.setPower(0.0); // Stop the motor
        }

        // Provide telemetry data to monitor motor power in real-time
        telemetry.addData("Motor Power", intakeMotor.getPower()); // Display current motor power
        telemetry.update(); // Push telemetry data to the driver station
    }
}
