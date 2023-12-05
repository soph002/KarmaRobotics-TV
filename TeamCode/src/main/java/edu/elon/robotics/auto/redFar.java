package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="red Far", group="labs")
public class redFar extends AutoCommon{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeArm();
        // close the gripper so you can put a block in to start
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_CLOSED);
        waitForStart();

        while(Math.abs(robot.motorArm.getCurrentPosition()) < 889 && opModeIsActive()) {
            robot.motorArm.setPower(robot.ARM_POWER_UP);
        }
        robot.motorArm.setPower(0);
        sleep(500);
        turnIMU(-90,.5);
        sleep(500);
        robot.resetDriveEncoders();
        driveIMU(70,.5);
        sleep(500);
        turnIMU(90,.5);
        sleep(500);
        robot.resetDriveEncoders();
        driveIMU(295,.7);
        sleep(500);
        turnIMU(-90,.5);
        sleep(500);
        robot.resetDriveEncoders();
        driveIMU(36,.5);

        int armTicksRed=889;
        double wristPosition=.1;

        while(Math.abs(robot.motorArm.getCurrentPosition()) > 689 && opModeIsActive()) {
            robot.motorArm.setPower(robot.ARM_POWER_DOWN);
        }
        System.out.println("hehe");
        robot.motorArm.setPower(0);
        robot.servoWrist.setPosition(wristPosition);
        sleep(500);
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        sleep(1000);
        while(Math.abs(robot.motorArm.getCurrentPosition()) < armTicksRed && opModeIsActive()) {
            robot.motorArm.setPower(robot.ARM_POWER_UP);
        }
        System.out.println("hehe");
        robot.motorArm.setPower(0);
        robot.resetDriveEncoders();
        driveIMU(85,-.5);
        System.out.println("here done");


    }

    /***********************************************************************/
    /** initializeArm is copied from the ArmBot code					  **/

    private void initializeArm() {

        // move the servo to known good positions
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        robot.servoWrist.setPosition(robot.WRIST_START_POS);

        // let the servos get to their position before moving the arm
        sleep(500);

        // tell the user that the arm is initializing
        telemetry.addData("ARM", "is initializing");
        telemetry.update();

        // initialize the arm
        robot.motorArm.setPower(robot.ARM_INIT_POWER);
        while (robot.touchSensor.getState()) {
            // do nothing -- waiting for a button press
        }
        robot.motorArm.setPower(0);

        // reset the encoder
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // tell the user we are done
        telemetry.addData("ARM", "initialization complete");
        telemetry.update();
    }

    /*******************************************************************************/
}
