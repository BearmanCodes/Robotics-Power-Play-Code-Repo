package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo", group="Cool OP modes")
public class Claw extends LinearOpMode {

    Servo servo;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    double retract = 0.20;
    double extend = 1;

    @Override
    public void runOpMode() {
        initialize(); //Configure and setup all the hardware, among other things

        waitForStart(); //Start the actual program once we confirm the robot isn't on fire

        //Do everything in these blocks until we press stop. Or something crashes.
        while (opModeIsActive()) {
            edgeDetector();
            if (gamepad1.a){
                servo.setPosition(extend);
            }
            if (gamepad1.b){
                servo.setPosition(retract);
            }
            telemetry.addData("Position: ", servo.getPosition());
            telemetry.update();
        }
    }

    private void initialize(){
        servo = hardwareMap.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(retract);
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
