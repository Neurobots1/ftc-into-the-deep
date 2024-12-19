import static android.os.Looper.loop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.ViperslidePIDF.target;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.SubMotor;

public class ViperslidePIDFBIS {

    private PIDController controller;

    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = -0.1;

    public static int target2 = 0;

    private final double ticks_in_degree = 384.5 / 180.0;
    public DcMotorEx slidemotorright;
    public DcMotorEx slidemotorleft;



    public void runOpMode(){

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");

        loop();
        {
            controller.setPID(p, i, d);
            int slidePos = slidemotorright.getCurrentPosition();
            int slidePos1 = slidemotorleft.getCurrentPosition();
            double pid = controller.calculate(slidePos, target2);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            slidemotorright.setPower(power);
            slidemotorleft.setPower(-power);
            telemetry.addData("pos", slidePos);
            telemetry.addData("pos1", slidePos1);
            telemetry.addData("target", target2);
            telemetry.addData("AmpR", slidemotorright.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("AmpL", slidemotorleft.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            if (gamepad1.a) {
                target2 = -50;
            }

            if (gamepad1.y){
                target2 = -2070;
            }

            if (gamepad1.b) {
                target2 = -900;
            }

            if (gamepad1.x) {
                target2 = -1525;
            }


        }
    }
}
