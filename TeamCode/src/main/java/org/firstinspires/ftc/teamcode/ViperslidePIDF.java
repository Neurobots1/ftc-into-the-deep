package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ViperslidePIDF extends OpMode {

    private PIDController controller;


    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = -0.1;

    public static int target = 0;
    public static int target1 = 0;

    private final double ticks_in_degree = 384.5 / 180.0;

    private DcMotorEx slidemotorright;
    private DcMotorEx slidemotorleft;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");
        slidemotorright.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            target = 2000;
            controller.setPID(p, i, d);
            int slidePos = slidemotorright.getCurrentPosition();
            int slidePos1 = slidemotorleft.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double pid1 = controller.calculate(slidePos1, target1);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double ff1 = Math.cos(Math.toRadians(target1/ticks_in_degree))*f;

            double power = pid + ff;
            double power1 = pid1 +ff1;


            slidemotorright.setPower(power);
            slidemotorleft.setPower(-power1);
            telemetry.addData("pos", slidePos);
            telemetry.addData("target", target);
            telemetry.update();
        } else {
            controller.setPID(p, i, d);
            int slidePos = slidemotorright.getCurrentPosition();
            int slidePos1 = -slidemotorleft.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double pid1 = controller.calculate(slidePos1, target1);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double ff1 = Math.cos(Math.toRadians(target1/ticks_in_degree))*f;

            double power = pid + ff;
            double power1 = pid1 +ff1;

            slidemotorright.setPower(power);
            slidemotorleft.setPower(-power1);
            telemetry.addData("pos", slidePos);
            telemetry.addData("target", target);
            telemetry.addData("target1", target1);
            telemetry.update();

            if (gamepad1.a){
                target = 50;
            }

            if (gamepad1.x){
                target = 900;
            }

            if (gamepad1.b){
                target = 1400;
            }
        }
    }
}