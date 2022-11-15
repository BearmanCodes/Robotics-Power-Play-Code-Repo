package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Super Cool OP Mode \uD83D\uDE0E", group="Cool OP modes")
public class CascadeTeleOp extends LinearOpMode {

    //Declare the variables
    private DcMotorEx motor;
    private DcMotorEx extend;
    DistanceSensor colorSensor;
    double distance;
    double extendPower;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(); //Configure and setup all the hardware, among other things

        waitForStart(); //Start the actual program once we confirm the robot isn't on fire

        //Do everything in these blocks until we press stop. Or something crashes.
        while (opModeIsActive()) {
            distance = colorSensor.getDistance(DistanceUnit.CM); //Update the color sensor distance right off the bat.
            extend.setPower(extendPower); //Set the extendable arm motor power to the variable.
            extendPower = -gamepad1.left_trigger * 0.45 + gamepad1.right_trigger * 0.45; //Set the power variable to the triggers.
            if (gamepad1.a) { //Do everything in here once you press A.
                motor.setPower(1); //Go full ham on the motor. (Wish I could use velocity but the encoder on the 40:1 is brokey)
                telemetry.addLine("It's a-moving!");
                telemetry.update();
                long start = System.currentTimeMillis(); //Declare and then set a variable to the current time
                long end = start + 1900; //Declare and then set a variable to what time to end (1900 MS after)
                while (System.currentTimeMillis() < end){ //While it still hasn't been 1900 MS since.
                    if (gamepad1.x) { //Press X to stop the motor in case something bad happens in between 1900 MS. The entire reason I used System instead of sleep.
                        motor.setPower(0);
                        break;
                    }
                    telemetry.addLine("It's a-moving!");
                    telemetry.addLine("Press X if things go Wacko!!!!!!!!!!!");
                    telemetry.update();
                }
                motor.setPower(0); //Stop the motor after it's moved for 1900 MS.
                telemetry.addLine("DONE!");
                telemetry.update();
            }

            if (gamepad1.b) { //Do everything in here once you press B.
                if (distance > 4.2) { //Makes sure it can only reverse if the cascading kit has moved up.
                    motor.setPower(-1); //Reverse, reverse. Cha cha now y'all.
                    distance = colorSensor.getDistance(DistanceUnit.CM); //Update the distance.
                    while (opModeIsActive() && distance > 4.2) { //While the cascading kit is still up do everything in here.
                        if (gamepad1.x) { //Press X to stop the motor in case everything goes WACKO!
                            motor.setPower(0);
                            break;
                        }
                        distance = colorSensor.getDistance(DistanceUnit.CM); //Update the distance within the loop, otherwise it'll just keep cha cha-ing.
                        telemetry.addLine("It's Cha Cha-ing!");
                        telemetry.update();
                    }
                    motor.setPower(0); //Stop once the cascading kit is down.
                }
            }
        }
    }

    private void initialize(){
        motor = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.
        motor.setDirection(DcMotorSimple.Direction.REVERSE); //Reverses it's direction so that it goes the right way.
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); //Assign the color sensor.
        extend = hardwareMap.get(DcMotorEx.class, "extend"); //Assign the extendable arm motor.
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set it to Brake like the cascading kit.
        extend.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse it's direction, subject to change.
        //These lines tighten the string just to make sure it's in the spool like it should be.
        motor.setPower(0.2);
        sleep(100);
        motor.setPower(0);
        telemetry.addLine("It didn't catch on fire ඞ"); //It passed the fire test. WOOOH!
        telemetry.update();
    }

    private void edgeDetector(){
        try{
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
        } catch(RobotCoreException e){
            telemetry.addLine("If you're seeing this, the program is trying to use a ga+"+
                    "mepad that does not exist. Fix it would ya?");
            telemetry.update();
        }
    }
}
