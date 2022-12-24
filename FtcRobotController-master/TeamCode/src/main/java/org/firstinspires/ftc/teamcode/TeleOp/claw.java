package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "claw", group="tests")
public class claw extends LinearOpMode {
    Servo lClaw;
    Servo rClaw;
    Gamepad previousGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();

    //Left claw, 0.1 opens it up, 0.3 closes it enough, 0.35/0.4 to really clamp.
    //Right claw, 0.0 opens it up, 0.2 closes it just a bit, 0.25/0.3 to really close.

    @Override
    public void runOpMode(){
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            try {
                edgeDetector();
            } catch (RobotCoreException e){
                telemetry.addLine("SOMETHING WENT WRONG");
                telemetry.update();
            }
            if (currentGamepad.cross && !previousGamepad.cross){ //Change to A once I get back
                lClaw.setPosition(0.4);
                rClaw.setPosition(0.3);
            }
            if (currentGamepad.circle && !previousGamepad.circle){
                lClaw.setPosition(0.1);
                rClaw.setPosition(0);
            }
        }

    }

    public void initialize(){
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");
        rClaw.setDirection(Servo.Direction.REVERSE);
        lClaw.setPosition(0.1);
        rClaw.setPosition(0);
    }

    public void edgeDetector() throws RobotCoreException{
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }

}
