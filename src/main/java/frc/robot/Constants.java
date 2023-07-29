// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class DriveTrain {
        public static final int leftFrontMotor = 1;
        public static final int leftRearMotor = 3;
        public static final boolean leftReversed = false;

        public static final int rightFrontMotor = 2;
        public static final int rightRearMotor = 4;
        public static final boolean rightReversed = true;

    }

    public static class Craw {
        public static final int motor = 1;
    }

    public static class Arm {
        public static final int motor = 0;
    }

    public static class Pneumatic {
        public static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        public static final int controller = 9;
        public static final int forwardChannel = 8;
        public static final int reverseChannel = 9;
    }
    

    public static class Sensor {

        public static final int encoderDistancePerPulse = 600;

        public static final int leftEncoderPort1 = 8;
        public static final int leftEncoderPort2 = 9;
        public static final boolean leftEncoderReversed = false;

        public static final int rightEncoderPort1 = 0;
        public static final int rightEncoderPort2 = 1;
        public static final boolean rightEncoderReversed = true;

        public static final int gyroPort = 0;
    }

    public static class IO {
        public static class Joystick {
            public static final int port = 0;

            public static final int portSpeed = 0;
            public static final int portRotation = 1;
        }

        public static class Controller {
            public static final int port  = 1;

        }
    }
}