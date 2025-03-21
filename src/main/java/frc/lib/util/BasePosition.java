package frc.lib.util;

public class BasePosition {
  /** Must be in the range of 0.0 to 1.0 */
  private double value;

  public BasePosition(double value) {
    if (value < 0.0) {
      throw new RuntimeException("ElevatorPosition cannot be less than 0.0");
    }
    if (value > 1.0) {
      throw new RuntimeException("ElevatorPosition cannot be more than 1.0");
    }
    this.value = value;
  }

  public double getValue() {
    return this.value;
  }

  /** Scales the inner range of 0.0 to 1.0 to the given range. */
  public double toRange(double lower, double upper) {
    double x1 = 0.0;
    double x2 = 1.0;
    double y1 = lower;
    double y2 = upper;
    double m = (y2 - y1) / (x2 - x1);

    // y = mx + b
    // y - mx = b
    // b = y - mx
    // b = y2 - m(x2)
    double b = y2 - m * x2;

    // y = mx + b
    double x = this.value;
    double y = m * x + b;

    return y;
  }

  public static BasePosition fromRange(double lower, double upper, double value) {
    if (value < lower) return new BasePosition(0.0);
    if (value > upper) return new BasePosition(1.0);

    double x1 = lower;
    double x2 = upper;
    double y1 = 0;
    double y2 = 1;
    double m = (y2 - y1) / (x2 - x1);
    double b = y1 - (m * x1);

    double x = value;
    double y = (m * x) + b;

    return new BasePosition(y);
  }
}
