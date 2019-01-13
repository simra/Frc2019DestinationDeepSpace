/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import frclib.FrcPixyCam;

public class RobotInfo
{
    //
    // Compiler switches
    //

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54*12.0;
    public static final double FIELD_WIDTH                      = 27*12.0;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Robot dimensions.
    //

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK              = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK             = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue

    public static final int CANID_PICKUP_MOTOR                  = 7;

    public static final int CANID_PDP                           = 16;
    public static final int CANID_PCM1                          = 17;
    public static final int CANID_PCM2                          = 18;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL        = 0;
    public static final int PDP_CHANNEL_LEFT_REAR_WHEEL         = 1;
    public static final int PDP_CHANNEL_RIGHT_REAR_WHEEL        = 14;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL       = 15;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //

    //
    // PWM Channels.
    //
    public static final int PWM_REV_BLINKIN                     = 0;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER               = 0;    // 20A: Purple

    //
    // Solenoid channels.
    //

    //
    // Vision subsystem.
    //
    public static final int PIXYCAM_WIDTH                       = 320;
    public static final int PIXYCAM_HEIGHT                      = 200;
    public static final int PIXY_BRIGHTNESS                     = 80;
    public static final double PIXY_CAM_OFFSET                  = 8.0;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.NORMAL_LANDSCAPE;
    public static final int PIXYCAM_I2C_ADDRESS                 = FrcPixyCam.DEF_I2C_ADDRESS;
    public static final int PIXY_TARGET_SIGNATURE               = 1;

    //
    // Ultrasonic sensors.
    //
    public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT              = 0.5;
    public static final double DRIVE_SLOW_XSCALE                = 3.0;
    public static final double DRIVE_SLOW_YSCALE                = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE             = 3.0;

    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second

    public static final double DRIVE_MAX_XPID_POWER             = 1.0;
    public static final double DRIVE_MAX_YPID_POWER             = 1.0;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.00874986860034917173913043478261;
    public static final double ENCODER_X_KP                     = 0.25;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.015;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 1.0;

    public static final double ENCODER_Y_INCHES_PER_COUNT       = 0.0176933159;
    public static final double ENCODER_Y_KP                     = 0.01;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.003;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;

    public static final double GYRO_TURN_KP                     = 0.02;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.00175;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

}   // class RobotInfo
