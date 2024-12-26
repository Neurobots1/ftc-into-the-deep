package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.GamePieceDetection;

@Config
@TeleOp
public class BlueTeleop extends OpMode {

    // Viper Slide Variables
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 360;
    private DcMotorEx slidemotorright;
    private DcMotorEx slidemotorleft;

    // Mecanum Drive Variables
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    IMU imu;
    public static double headingKp = -0.005; // Proportional constant for heading correction
    public static double headingSetpoint = 0;  // Desired heading
    private double headingError;         // The difference between current heading and desired heading
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;  // Declare the touch sensor

    private DcMotor intakemotor;
    private ColorSensor colorSensor;
    private GamePieceDetection gamePieceDetection;
    private boolean limitSwitchPreviouslyPressed = false;

    @Override
    public void init() {
        // Initialize GamePieceDetection
        gamePieceDetection = new GamePieceDetection(hardwareMap.get(ColorSensor.class, "colorSensor"));

        // Initialize Viper Slide
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");
        slidemotorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidemotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Mecanum Drive Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // Set motor behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize REV Touch Sensor (Limit Switch)
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch"); // Assign the touch sensor

        // Initialize motors and sensors
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize GamePieceDetection
        gamePieceDetection = new GamePieceDetection(colorSensor);
    }

    @Override
    public void loop() {

        // Update the game piece color detection
        gamePieceDetection.detectColor();
        String detectedColor = gamePieceDetection.getDetectedColor();

        // Check if the detected color is the opponent's color (assuming the opponent's color is Red)
        if (detectedColor.equals("Red")) {
            // Opponent's color detected, outake immediately at 0.3 power
            intakemotor.setPower(0.3);
        } else {
            // Check if left or right bumper is pressed for intake/outtake control
            if (gamepad1.left_bumper) {
                // Full intake power when left bumper is pressed
                intakemotor.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                // Outtake at 0.3 power when right bumper is pressed
                intakemotor.setPower(-0.3);
            } else {
                // Stop motor if no bumpers are pressed
                intakemotor.setPower(0);
            }
        }

        // Viper Slide Control (PID)
        controller.setPID(p, i, d);
        int slidePosLeft = slidemotorleft.getCurrentPosition();
        int slidePosRight = slidemotorright.getCurrentPosition();
        double pidLeft = controller.calculate(slidePosLeft, target);
        double pidRight = controller.calculate(slidePosRight, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double powerLeft = pidLeft + ff;
        double powerRight = pidRight + ff;

        slidemotorleft.setPower(powerLeft);
        slidemotorright.setPower(powerRight);

        // Button presses to change target for the Viper slide
        if (gamepad1.y) {
            target = 2950;
        }
        if (gamepad1.a) {
            target = 0;
        }
        if (gamepad1.x) {
            target = 900;
        }
        if (gamepad1.b) {
            target = 1400;
        }

        // **Limit Switch Functionality (Debounce)**
        boolean isLimitSwitchPressed = limitSwitch.isPressed();
        if (isLimitSwitchPressed && !limitSwitchPreviouslyPressed) {
            // Reset encoders to zero
            slidemotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target position to 0
            slidemotorleft.setTargetPosition(0);
            slidemotorright.setTargetPosition(0);

            // Re-enable running without encoders after reset
            slidemotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidemotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Limit Switch", "Resetting encoders");
        }

        // Update the state variable
        limitSwitchPreviouslyPressed = isLimitSwitchPressed;

        // Mecanum Drive Control with Heading Lock
        double y = -gamepad1.left_stick_y; // Y-axis (reversed)
        double x = gamepad1.left_stick_x;  // X-axis (strafing)
        double rx = gamepad1.right_stick_x; // Rotation

        // Reset IMU yaw if the options button is pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Get current robot heading (yaw) from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Update the heading setpoint when there is manual rotation input
        if (Math.abs(rx) > 0.05) {
            headingSetpoint = botHeading;
        }

        // Calculate the heading error and apply proportional control
        headingError = headingSetpoint - botHeading;
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
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Telemetry for debugging and visualization
        telemetry.addData("posLeft", slidePosLeft);
        telemetry.addData("posRight", slidePosRight);
        telemetry.addData("target", target);
        telemetry.addData("Heading", botHeading);
        telemetry.addData("Target Heading", headingSetpoint);
        telemetry.addData("Heading Error", headingError);

        // Graphing current and target heading for real-time visualization on the dashboard
        dashboard.getTelemetry().addData("Current Heading", botHeading);
        dashboard.getTelemetry().addData("Target Heading", headingSetpoint);
        dashboard.getTelemetry().addData("Heading Error", headingError);

        telemetry.update();
    }


}
