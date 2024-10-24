package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "mecanum (Blocks to Java)")
public class mecanum extends LinearOpMode {

    private DcMotor FrontR;
    private DcMotor FrontL;
    private DcMotor RearR;
    private DcMotor RearL;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        RearR = hardwareMap.get(DcMotor.class, "RearR");
        RearL = hardwareMap.get(DcMotor.class, "RearL");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put loop blocks here.
                FrontR.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
                FrontL.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x));
                RearR.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x));
                RearL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                FrontR.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
                FrontL.setPower(-(-gamepad1.right_stick_y + gamepad1.right_stick_x));
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                RearR.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
                RearL.setPower(-(-gamepad1.right_stick_y + gamepad1.right_stick_x));
            }
        }
    }}

