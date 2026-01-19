package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.OperatorConstants.*;

@Logged
public class Controls {

    private static Controls singleton;

    static SendableChooser<DriveType> driveTypeChooser = new SendableChooser<DriveType>();

    @NotLogged
    CommandXboxController controller = new CommandXboxController(0);
    @NotLogged
    CommandXboxController driverController = new CommandXboxController(3);
    @NotLogged
    CommandJoystick rightJoystick = new CommandJoystick(1);
    @NotLogged
    CommandJoystick leftJoystick = new CommandJoystick(2);


    DriveType driveType = DriveType.JOYSTICK;

    private Controls() {
        DriverStation.silenceJoystickConnectionWarning(true);
        driveTypeChooser.addOption("JOYSTICK", DriveType.XBOX);
        driveTypeChooser.setDefaultOption("XBOX", DriveType.JOYSTICK);
        SmartDashboard.putData(driveTypeChooser);

        driverController.start().onTrue(Drivetrain.getInstance().zeroHeadingCommand());
        leftJoystick.button(4).onTrue(Drivetrain.getInstance().zeroHeadingCommand());
        rightJoystick.button(4).onTrue(Drivetrain.getInstance().zeroHeadingCommand());
        
        Drivetrain.getInstance().setDefaultCommand(new Drive());          
    }
    
    public static Controls getInstance() {
        if (singleton == null) {
            singleton = new Controls();
        }
        return singleton;
    }

    @Logged 
    public double[] getSwerveJoystickValues() {
        double slowmodePercent = getSlowmode() ? 1 : .67;
        
        switch (getDriveType()) {
            case JOYSTICK -> {
                return new double[] {
                        // applyDeadband will do the absolute value stuff for us and make the zero point
                        // start at
                        // the deadzone edge
                        Math.pow(-MathUtil.applyDeadband(leftJoystick.getY(), driverJoystickDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(leftJoystick.getX(), driverJoystickDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(rightJoystick.getX(), driverJoystickDeadzone), 1),
                };
            }
            case XBOX -> {
                return new double[] {
                        // applyDeadband will do the absolute value stuff for us and make the zero point
                        // start at
                        // the deadzone edge
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftY(), xboxControllerDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftX(), xboxControllerDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(
                                -MathUtil.applyDeadband(driverController.getRightX(), xboxControllerDeadzone), 1),
                };
            }
            default -> throw new IllegalStateException("Unexpected value: " + getDriveType());
        }
    }

    private boolean getSlowmode() {
        return leftJoystick.getHID().getRawButton(3) || rightJoystick.getHID().getRawButton(3);
    }

    public DriveType getDriveType() {
        driveType = driveTypeChooser.getSelected();
        return driveType;
    }

    public enum DriveType {
        XBOX,
        JOYSTICK
    }

    public Command rumbleCommand() {
        return new RunCommand(() -> {
            controller.setRumble(RumbleType.kBothRumble, 1);
        }).finallyDo(() -> {
            controller.setRumble(RumbleType.kBothRumble, 0);
        });
    }
}
