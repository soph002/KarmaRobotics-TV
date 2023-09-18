package edu.elon.robotics.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Encoder Drive", group="labs")
public class EncoderDrive extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        robot.resetDriveEncoders();
        robot.startMove(0, 0.3, 0, 0);
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < 3000) {

        }
        robot.startMove(0,0,0,0);
        System.out.println("MYDATA: " + robot.motorLeft.getCurrentPosition() + "," +
                robot.motorRight.getCurrentPosition() + "," + robot.motorAux.getCurrentPosition());


    }
}
