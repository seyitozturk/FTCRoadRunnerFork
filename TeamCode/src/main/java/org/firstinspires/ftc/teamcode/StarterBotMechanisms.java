package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Everything on the goBILDA StarterBot that is NOT the drivetrain: intake roller, two corner
 * intake servos, the windmill feeder servo and the launcher flywheel. The drivetrain lives in
 * SampleMecanumDrive (Road Runner). TeleOp and Autonomous both use this class so the launcher
 * behaves the same in both.
 *
 * Hardware names match the goBILDA StarterBot example configuration.
 *
 * @Config lets you change these values live from FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@Config
public class StarterBotMechanisms {
    // Launcher speeds in encoder ticks/second (1:1 motor, 28 ticks/rev -> 1250 tps = ~2678 RPM)
    public static double LAUNCHER_TARGET_VELOCITY = 1250;
    public static double LAUNCHER_MIN_VELOCITY = 1200;
    public static PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    private final DcMotor intake;
    private final DcMotorEx launcher;
    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;
    private final CRServo windmillServo;

    public StarterBotMechanisms(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
       // windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        stop();
    }

    /**
     * Call once per loop.
     *
     * @param intakePower -1..1, positive pulls elements in
     * @param launch      true = spin up the launcher, and feed once it is up to speed
     */
    public void update(double intakePower, boolean launch) {
        if (launch) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (launch && isLauncherReady()) {
            windmillServo.setPower(1);
            intakePower += 0.5; // helps push stuck elements up into the launcher
        } else {
            windmillServo.setPower(0);
        }

        intakePower = Math.max(-1, Math.min(1, intakePower));
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    public boolean isLauncherReady() {
        return launcher.getVelocity() > LAUNCHER_MIN_VELOCITY;
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public void stop() {
        update(0, false);
    }
}
