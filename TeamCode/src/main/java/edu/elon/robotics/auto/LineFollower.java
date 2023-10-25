package edu.elon.robotics.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Line Follower Drive", group="labs")
public class LineFollower extends AutoCommon{

    protected void calibrateColorSensor(double power){
        int smallest=10000;
        int largest=0;
        robot.resetDriveEncoders();
        // turn left 45 degrees
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDegreesToTicks(45) && opModeIsActive()){
            robot.startMove(0,0,-power,0);
            System.out.println("brightness"+ robot.colorSensor.alpha());
            if(robot.colorSensor.alpha()<smallest){
                smallest=robot.colorSensor.alpha();
            }
            else if(robot.colorSensor.alpha()>largest){
                largest=robot.colorSensor.alpha();
            }
        }
        robot.startMove(0,0,0,0);
        sleep(500);
        System.out.println("brightness smallest after turn 1 is "+ smallest+"brightness largest"+largest);
        robot.resetDriveEncoders();
        // turn right 90 degrees
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDegreesToTicks(90) && opModeIsActive()){
            robot.startMove(0,0,power,0);
            System.out.println("brightness"+ robot.colorSensor.alpha());
            if(robot.colorSensor.alpha()<smallest){
                smallest=robot.colorSensor.alpha();
            }
            else if(robot.colorSensor.alpha()>largest){
                largest=robot.colorSensor.alpha();
            }
        }
        robot.startMove(0,0,0,0);
        sleep(500);
        System.out.println("brightness smallest after turn 2 is "+ smallest+"brightness largest"+largest);
        robot.resetDriveEncoders();
        // turn left 45 degrees
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDegreesToTicks(45) && opModeIsActive()){
            robot.startMove(0,0,-power,0);
            System.out.println("brightness"+ robot.colorSensor.alpha());
            if(robot.colorSensor.alpha()<smallest){
                smallest=robot.colorSensor.alpha();
            }
            else if(robot.colorSensor.alpha()>largest){
                largest=robot.colorSensor.alpha();
            }
        }
        System.out.println("brightness smallest after turn 3 is "+ smallest+"brightness largest"+largest);
        robot.startMove(0,0,0,0);
        sleep(500);
        // brightness smallest100brightness largest3660
        System.out.println("brightness smallest"+ smallest+"brightness largest"+largest);

        robot.maxBrightness=largest;
        robot.minBrightness=smallest;
    }

    protected void pController(double power){
        ElapsedTime timer=new ElapsedTime();
        timer.reset();
        double desiredLightValue= (robot.maxBrightness+robot.minBrightness)*.5;
        System.out.println("DESIRED IS"+desiredLightValue);
        robot.startMove(power,0,0,0);
        while(opModeIsActive()){
            double value=robot.colorSensor.alpha();
            System.out.println("P-Control: "+timer.milliseconds()+","+value);
            double curLightIntensity=robot.colorSensor.alpha();
            System.out.println("currentLightIntensity"+curLightIntensity);
            double error=curLightIntensity-desiredLightValue;
            System.out.println("error is"+error);
            double maxError=(robot.maxBrightness - robot.minBrightness)/2.0;
            double Kp=(1.0/maxError)*0.5;
            double turn = Kp*error;
            System.out.println("turn is "+turn);
            robot.startMove(power,0,turn,0);
        }
    }
    protected void pidController(double power){
        double desiredLightValue= (robot.maxBrightness+robot.minBrightness)*.5;
        ElapsedTime loopTimer = new ElapsedTime();
        robot.startMove(power,0,0,0);
        double sumError=0;
        double prevError=0;
        double KP=.00092937*.6;
        double dT=.03;
        double Pc=.30176470588;
        double KI=(2.0*KP) * dT / Pc;
        double KD=KP * Pc / (8*dT);
        while(opModeIsActive()){
            loopTimer.reset();
            double curLightIntensity=robot.colorSensor.alpha();
            double error=curLightIntensity-desiredLightValue;
            sumError=sumError + error;
            double diffError=error-prevError;

            double turn = (KP*error)+(KI*sumError)+(KD*diffError);
            System.out.println("error is"+error);
            System.out.println("turn is"+turn);
            prevError=error;
            sleep(30 - Math.round(loopTimer.milliseconds()));
            robot.startMove(power,0,turn,0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        calibrateColorSensor(.3);
        sleep(500);
        pidController(.3);
    }
}
