package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Purple Burglar Alarm \uD83D\uDE0E", group="Cool OP modes")
public class Cascade extends LinearOpMode {

    //Declare the variables
    private DcMotorEx motor;
    private DcMotorEx extend;
    DistanceSensor colorSensor;
    double distance;
    double cascadePower;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(); //Configure and setup all the hardware, among other things

        waitForStart(); //Start the actual program once we confirm the robot isn't on fire

        //Do everything in these blocks until we press stop. Or something crashes.
        while (opModeIsActive()) {
            distance = colorSensor.getDistance(DistanceUnit.CM); //Update the color sensor distance right off the bat.
            cascadePower = -gamepad1.left_trigger + gamepad1.right_trigger;
            if(distance > 4.2 || cascadePower > 0){
                motor.setPower(cascadePower);
            }else{
                motor.setPower(0);
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
            telemetry.addLine("If you're seeing this, the program is trying to use a ga"+
                    "mepad that does not exist. Fix it would ya?");
            telemetry.update();
        }
    }
}
