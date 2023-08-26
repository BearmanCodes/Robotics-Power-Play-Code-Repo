package org.firstinspires.ftc.teamcode.TeleOp.RequiesceToo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Claw extends LinearOpMode {
    RequiesceToo gpad = new RequiesceToo();
    boolean clawToggle = false;
    boolean flipToggle = false;
    Servo flipper, lClaw, rClaw;

    public void Go(){
        //Left claw, 0.1 opens it up, 0.3 closes it enough, 0.35/0.4 to really clamp.
        //Right claw, 0.0 opens it up, 0.2 closes it just a bit, 0.25/0.3 to really close.
        try {
            gpad.edgeDetector();
        } catch (RobotCoreException e){
            telemetry.addLine("WHAT HAPPEN");
            telemetry.update();
        }
        if (gpad.currentGamepad.a && !gpad.previousGamepad.a){ //Change to A once I get back
            clawToggle = !clawToggle;
        }
        if (clawToggle){
            lClaw.setPosition(0.35);
            rClaw.setPosition(0.25);
        }
        else {
            lClaw.setPosition(0.2);
            rClaw.setPosition(0.05);
        }
        if (gpad.currentGamepad.b && !gpad.previousGamepad.b){ //Change to B once I get back
            flipToggle = !flipToggle;
        }
        if (flipToggle){
            flipper.setPosition(-0.02);
        }
        else {
            flipper.setPosition(0.33);
        }
    }

    public void Init(){
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");
        flipper = hardwareMap.get(Servo.class, "flip");


        rClaw.setDirection(Servo.Direction.REVERSE);
        lClaw.setPosition(0.2);
        flipper.setPosition(0.33);
        rClaw.setPosition(0.05);
    }

    public void runOpMode() throws InterruptedException {
    }
}
