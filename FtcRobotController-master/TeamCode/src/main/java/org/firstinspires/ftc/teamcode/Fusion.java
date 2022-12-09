package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Disabled.Cascade;

@TeleOp(name="Garrett Drive", group="Garrett")
public class Fusion extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx arm;
    private DcMotorEx coreHex;
    DistanceSensor colorSensor;
    double hexPower;
    double cascadePower;
    double distance;
    double extendPower;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    BNO055IMU imu;
    double backRightPower;
    double frontRightPower;
    double backLeftPower;
    double frontLeftPower;
    double Vertical;
    double Horizontal;
    double Pivot;
    public Cascade cascade = new Cascade();



    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            hexPower = -gamepad1.left_trigger + gamepad1.right_trigger;
            coreHex.setPower(hexPower);
            distance = colorSensor.getDistance(DistanceUnit.CM); //Update the color sensor distance right off the bat.
            cascadePower = -gamepad1.left_trigger + gamepad1.right_trigger;
            if(distance > 4.2 || cascadePower > 0){
                arm.setPower(cascadePower);
            }else{
                arm.setPower(0);
            }
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.2;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double frontLeftPower = (rotY + rotX + rx) * 0.75;
            double backLeftPower = (rotY - rotX + rx) * 0.75;
            double frontRightPower = (rotY - rotX - rx) * 0.75;
            double backRightPower = (rotY + rotX - rx) * 0.75;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }

    private void initialize(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.
        arm.setDirection(DcMotorSimple.Direction.REVERSE); //Reverses it's direction so that it goes the right way.
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); //Assign the color sensor.
        //extend = hardwareMap.get(DcMotorEx.class, "extend"); //Assign the extendable arm motor.
        //extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set it to Brake like the cascading kit.
        //extend.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse it's direction, subject to change.
        coreHex = hardwareMap.get(DcMotorEx.class, "hex");
        //These lines tighten the string just to make sure it's in the spool like it should be.
        arm.setPower(0.2);
        sleep(100);
        arm.setPower(0);
        telemetry.addLine("It didn't catch on fire ඞ"); //It passed the fire test. WOOOH!
        telemetry.update();

    }
}
