package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.SubDependensy.SubMotor.target;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class IteratifTeleop extends OpMode {


 @Override
    public void init() {
       // Declare our motors
       // Make sure your ID's match your configuration
       DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
       DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
       DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
       DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


       // Reverse the right side motors. This may be wrong for your setup.
       // If your robot moves backwards when commanded to go forwards,
       // reverse the left side instead.
       // See the note about this earlier on this page.
       frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

       // Retrieve the IMU from the hardware map
       IMU imu = hardwareMap.get(IMU.class, "imu");
       // Adjust the orientation parameters to match your robot
       IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
               RevHubOrientationOnRobot.LogoFacingDirection.UP,
               RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
       // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
       imu.initialize(parameters);


    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


       }



    }
