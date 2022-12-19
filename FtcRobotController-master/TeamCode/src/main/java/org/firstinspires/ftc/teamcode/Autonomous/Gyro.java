package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Gyro", group="TESTS")
public class Gyro extends LinearOpMode {
    private final DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
    private final DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
    private final DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
    private final DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    int frontRightTarget;
    int frontLeftTarget;
    int backRightTarget;
    int backLeftTarget;
    double robotHeading = 0;
    double headingOffset = 0;
    double headingError = 0;
    DcMotorEx[] allMotors = {frontLeft, frontRight, backLeft, backRight};
    BNO055IMU imu;

    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);

    @Override
    public void runOpMode(){
        initialize();

        waitForStart();

        allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();
    }

    public void straight(double maxDriveSpeed,
                         double distance,
                         double heading){
        if (opModeIsActive()){
            frontRightTarget = frontRight.getCurrentPosition() + (int) (distance * TicksPerIn);
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * TicksPerIn);
            backRightTarget = backRight.getCurrentPosition() + (int) (distance * TicksPerIn);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (distance * TicksPerIn);

            setTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            maxDriveSpeed = Math.abs(maxDriveSpeed)
        }
    }

    private void allMotorMode(DcMotor.RunMode mode){
        for (DcMotorEx motor: allMotors){
            motor.setMode(mode);
        }
    }

    private void allMotorPower(double power){
        for (DcMotorEx motor : allMotors){
            motor.setPower(power);
        }
    }

    private void setTargetPosition(int frontLeftPos, int frontRightPos,
                                   int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading(){
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    private void initialize() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
