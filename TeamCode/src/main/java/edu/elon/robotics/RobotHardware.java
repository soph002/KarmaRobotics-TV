package edu.elon.robotics;

/**
 * Defines the robot hardware and implements a few
 * fundamental methods.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;

    public RobotHardware(HardwareMap hardwareMap) {
        /*
         * This code provides an object to control the physical
         * motor referenced by the configuration string.  The
         * positive direction of rotation is established. Finally,
         * the motor is directed to forcefully stop when no power
         * is applied.
         */

        // define the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorAux = hardwareMap.dcMotor.get("motorAux");
        motorAux.setDirection(DcMotorSimple.Direction.FORWARD);
        motorAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset the drive encoders to zero
        resetDriveEncoders();
    }

    public void resetDriveEncoders() {
        /*
         * This code resets the encoder values back to 0 for
         * each of the three drive motors.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAux.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startMove(double drive, double strafe, double turn, double modifier) {
        /*
         * How much power should we apply to the left,
         * right, and aux motor?
         *
         * If all 3 motors apply the same power in the
         * same direction, the robot will turn in place.
         */
        // reqPower = 0.8	// requested power
        //	BASE_POWER = 0.15	// power at which the robot starts to move
        //	RAMP_TICKS = 1000 	// how many ticks to ramp
        //	curTicks 		// current number of ticks from the motor
        //	curPower		// current power of the motor
        //
        //	// ramp up to the requested power
        //	curPower = basePower
        //	while (curTicks is less than RAMP_TICKS)
        //	  set motor power to curPower
        //  curPower = BASE_POWER + (curTicks / RAMP_TICKS * (reqPower - BASE_POWER))
        double requestPower=drive+strafe+turn;
        double RAMP_TICKS = 500;
        double BASE_POWER = .4;
        double curPower=BASE_POWER;

        motorLeft.setPower(BASE_POWER);
        motorRight.setPower(-BASE_POWER);
        motorAux.setPower(0);

        while(Math.abs(motorLeft.getCurrentPosition()) < RAMP_TICKS){
            motorLeft.setPower(curPower);
            motorRight.setPower(-curPower);
            motorAux.setPower(0);
            curPower=BASE_POWER+(Math.abs(motorLeft.getCurrentPosition())/RAMP_TICKS * (requestPower-BASE_POWER));
        }


        double powerLeft= 0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        double powerRight= -0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        double powerAux= -2.0 * strafe / 3.0 + turn / 3.0;

        double scale = Math.max(Math.max(Math.max(1.0, Math.abs(powerLeft)), Math.abs(powerRight)), Math.abs(powerAux));

        powerLeft=powerLeft/scale;
        powerRight=powerRight/scale;
        powerAux=powerAux/scale;
        /*
         * Apply the power to the motors.
         */
        motorLeft.setPower(powerLeft);
        motorRight.setPower(powerRight);
        motorAux.setPower(powerAux);
    }

}
