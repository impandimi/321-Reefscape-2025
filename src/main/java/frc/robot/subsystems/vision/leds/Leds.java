/* (C) Robolancers 2025 */
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/vision/leds/Leds.java
package frc.robot.subsystems.vision.leds;
========
package frc.robot.subsystems.leds;
>>>>>>>> d97284cec42ed898829e77e33d67d5d9784ab119:src/main/java/frc/robot/subsystems/leds/Leds.java

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Leds extends VirtualSubsystem {
  // priority (greater # means more important), condition for pattern to apply, pattern to apply
  public record Signal(int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {}

  // LED
  public final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private static TreeSet<Signal> ledSignals;
  private LEDPattern currentPattern = LEDPattern.kOff;

  // Constants
  private final int length = 30;
  private final Color algaeColor = new Color(133, 226, 203);

  // Patterns
  // default mode - meteor yellow. TODO: add meteor pattern or something cool like that
  public LEDPattern kDefault = LEDPattern.kOff;

  // climb mode - solid blue
  public LEDPattern kClimbing = LEDPattern.solid(Color.kBlue);

  // aligning - solid red
  public LEDPattern kAligning = LEDPattern.solid(Color.kRed);

  // aligned - solid green
  public LEDPattern kAligned = LEDPattern.solid(Color.kGreen);

  // has coral - solid white
  public LEDPattern kHasCoral = LEDPattern.solid(Color.kWhite);

  // has algae - solid algaeColor
  public LEDPattern kHasAlgae = LEDPattern.solid(algaeColor);

  // has coral and algae - gradient of pink & light green (patrick star)
  private LEDPattern hasCoralAndAlgaePattern =
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kHotPink, new Color(172, 220, 65));
  public LEDPattern kHasCoralAndAlgae =
      hasCoralAndAlgaePattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.2));

  // Intaking - solid orange
  public LEDPattern kIntaking = LEDPattern.solid(Color.kOrange);

  // Outtaking - blinking orange
  public LEDPattern kOuttaking = kIntaking.blink(Seconds.of(0.5));

  public Leds() {
    ledStrip = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(length);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // sorted in descending order of priority
    ledSignals = new TreeSet<Signal>(Comparator.comparingInt(Signal::priority).reversed());
  }

  // register signals into the tree set. If priority is already taken, don't add it
  public static void registerSignal(
      int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {
    final var priorityIsAlreadyClaimed =
        ledSignals.stream().mapToInt(Signal::priority).anyMatch(p -> p == priority);

    if (priorityIsAlreadyClaimed) {
      DriverStation.reportWarning("Priority " + priority + " is already claimed", true);
      return;
    }

    ledSignals.add(new Signal(priority, condition, pattern));
  }

  @Override
  public void periodic() {
    // set new pattern to the pattern of the most important signal that is true
    for (var signal : ledSignals) {
      if (signal.condition.getAsBoolean()) {
        currentPattern = signal.pattern.get();
        break;
      }
    }

    // apply the pattern to the led strip
    currentPattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  
}
