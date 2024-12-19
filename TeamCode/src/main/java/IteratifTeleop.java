import static org.firstinspires.ftc.teamcode.SubMotor.target;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubMotor;

@TeleOp
public class IteratifTeleop extends OpMode {
    // SubMotor target = new target(0);

   @Override
    public void init() {


    }
    @Override
    public void loop() {
       if (gamepad1.a){
           target=-50;
       }



    }

}
