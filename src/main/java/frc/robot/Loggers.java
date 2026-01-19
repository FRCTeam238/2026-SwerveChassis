package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class Loggers {
    @CustomLoggerFor(PositionVoltage.class)
    public static class PositionVoltageLogger extends ClassSpecificLogger<PositionVoltage> {
        public PositionVoltageLogger() {
            super(PositionVoltage.class);
        }

        public void update(EpilogueBackend backend, PositionVoltage voltage) {
            backend.log("Feedforward", voltage.FeedForward);
            backend.log("Position", voltage.Position);
        }
    }
}
