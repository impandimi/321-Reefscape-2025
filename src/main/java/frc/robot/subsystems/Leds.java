/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

  // Patterns
  public LEDPattern kRedAlliance = LEDPattern.solid(Color.kRed);
  public LEDPattern kBlueAlliance = LEDPattern.solid(Color.kBlue);
  public LEDPattern kRainbow = LEDPattern.rainbow(255, 255);
  public LEDPattern kScrollingRainbow =
      kRainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 120.0));

  public Leds() {
    ledStrip = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(30);
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
