package org.firstinspires.ftc.teamcode.THISIS10111.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.Robot10111;
import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.motors.CoolMotor101;

///finished editing in from prev yr
//import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.HowlersHardware;

//import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class DriveTrain101 extends SubsystemBase {
    @Config
    public static class DriveTrainConstants {
        public static PIDCoefficients FORWARD_PID = new PIDCoefficients(0.00075,0,0);
        public static double FORWARD_kStatic = 0.1;
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.0025,0,0);
        public static double HEADING_kStatic = 0.0125;
    }


    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    private MecanumDrive driveTrain;


    private DriveMode driveMode;
    public DriveMode setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        return this.driveMode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }
    public enum DriveMode {
        MANUAL,
        AUTOIDLE,
        AUTOFORWARD,
        AUTOTURN,
    }



    private double speed = 0.6;
    public double setSpeed(double setter) {
        speed = setter;
        return speed;
    }
    public double getSpeed() {
        return speed;
    }
//



    //variables to get
    public double manualForward = 0;
    private double manualTurn = 0;
    private double manualStrafe = 0;
    private double manualArm = 0;



    double forwardCalculation;
    double turnCalculation;
    double strafeCalculation;

    double armY;
    public PIDFController forwardPID;
    public PIDFController headingPID;
    double targetAngle = 0;
    double targetX = 0;

    //int count = 0;

    Orientation angles;

    public DriveTrain101(final HardwareMap hwMap, DriveMode driveMode) {
        Robot10111 robot = Robot10111.getInstance();
        CommandScheduler.getInstance().registerSubsystem(DriveTrain101.this);

        this.driveMode = driveMode;
        forwardPID = new PIDFController(DriveTrainConstants.FORWARD_PID,0,0, DriveTrainConstants.FORWARD_kStatic);
        headingPID = new PIDFController(DriveTrainConstants.HEADING_PID,0,0, DriveTrainConstants.HEADING_kStatic);

        robot.rightFront = new CoolMotor101(hwMap, "rightFront", 134.4);
        robot.rightBack = new CoolMotor101(hwMap, "rightBack", 134.4);
        robot.leftBack = new CoolMotor101(hwMap, "leftBack", 134.4);
        robot.leftFront = new CoolMotor101(hwMap, "leftFront", 134.4);
        robot.leftBack.setInverted(true);
        robot.leftFront.setInverted(true);
        robot.rightBack.setInverted(true);
        robot.rightFront.setInverted(true);
        //leftMotors = new MotorGroup(robot.rightFront, robot.rightBack);
        //rightMotors = new MotorGroup(robot.leftBack, robot.leftFront);

        //robot.leftFront.setInverted(true); //switch based on blocks
        //robot.leftBack.setInverted(true);
        //robot.rightBack.setInverted(true);
        //robot.rightFront.setInverted(true);
        //driveTrain = new DifferentialDrive(leftMotors, rightMotors);
        driveTrain = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
        driveTrain.setMaxSpeed(0.8);

    }
    public void setManualDrive(double strafe, double forward, double turn) {
        //manually driving,specifiying amt of strafing, forward, and turn
        manualForward = forward;
        manualTurn = turn;
        manualStrafe = strafe;
    }
    public double getForwardCalculation(){

        return forwardCalculation;
    }
    public double getStrafeCalculation(){

        return strafeCalculation;
    }
    public double getTurnCalculation(){

        return turnCalculation;
    }
    public double getManualForward(){

        return manualForward;
    }
    public double getManualTurn(){

        return manualTurn;
    }
    public double getManualStrafe(){

        return manualStrafe;
    }
    /*public int getCount() {

        return count;
    }*/
    public void update() {

        //count++;
        Robot10111 robot = Robot10111.getInstance();

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        switch (driveMode) {
            case MANUAL:
                forwardCalculation = manualForward;
                turnCalculation = manualTurn;
                strafeCalculation = manualStrafe;
                armY = manualArm;
                break;
            case AUTOFORWARD:
                forwardCalculation = forwardPID.update(robot.leftFront.getEncoderCount());
                turnCalculation = 0;
                strafeCalculation = 0;
                armY = 0;
                break;
            case AUTOTURN:
                forwardCalculation = 0;
                turnCalculation = headingPID.update(currentHeading());
                strafeCalculation = 0;
                armY = 0;
                break;
            /*default:
                forwardCalculation = 0;
                turnCalculation = 0;
                strafeCalculation = 0;
                break;*/
        }

        driveTrain.driveRobotCentric(strafeCalculation, forwardCalculation, turnCalculation);

        // run arm up method using armY
        robot.arm.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void setHeading(double target) {

        // sets where the robot is currently headed : angle
        targetAngle = target;
        headingPID.setTargetPosition(target);
    }

    public double currentHeading() {

        // seems to return a turn angle, not sure what
        Robot10111 robot = Robot10111.getInstance();

        double currentHeading;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //if(angles.firstAngle <= 0) currentHeading = 360 - Math.abs(angles.firstAngle);
        currentHeading = -angles.firstAngle;

        return currentHeading;
    }
    public void setX(double target) {
        //sets
        this.targetX = target;
        forwardPID.setTargetPosition(target);

    }

    //@Override
    public void periodic() {

        update();
        //System.out.println("new update");
    }

    public void resetEncoders() {
        Robot10111 robot = Robot10111.getInstance();

        robot.leftBack.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightFront.resetEncoder();

    }

    public void slow() { driveTrain.setMaxSpeed(0.25); }
    public void fast() {
        driveTrain.setMaxSpeed(0.75);
    }


    public void stop()
    {
        Robot10111 robot = Robot10111.getInstance();

        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);

    }

   /* public void drive(double forward, double turn) {
        driveTrain.arcadeDrive(forward, turn);
    }
*/
    public boolean isBusy() {
        Robot10111 robot = Robot10111.getInstance();
        boolean isBusy;
        if(robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy()) isBusy = true;
        else isBusy = false;
        return isBusy;
    }

}