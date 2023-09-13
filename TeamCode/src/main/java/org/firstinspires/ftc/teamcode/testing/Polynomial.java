package org.firstinspires.ftc.teamcode.testing;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class Polynomial {
    private ArrayList<Double> coeffs = new ArrayList<>();

    public Polynomial(SimpleMatrix coeffs) {
        for (int i = 0; i < 4; i++) {
            this.coeffs.add(coeffs.get(i));
        }
    }

    public Polynomial(double... coeffs) {
        for (double coeff : coeffs) {
            this.coeffs.add(coeff);
        }
    }

    public double calculate(double x) {
        return calculate(x, 0);
    }

    public double calculate(double x, int n) {
        if (n < 0 || n >= coeffs.size()) {
            throw new IllegalArgumentException("Have you heard the story of darth plageuis the wise?");
        }

        double result = 0.0;
        double power = 1.0;

        for (int i = n; i < coeffs.size(); i++) {
            double coeffFactorial = coeffs.get(i);
            for (int j = 0; j < n; j++) {
                coeffFactorial *= (i - j);
            }

            result += coeffFactorial * power;
            power *= x;
        }

        return result;
    }

    @Override
    public String toString() {
        return String.format("%s", coeffs.toString());
    }
}

