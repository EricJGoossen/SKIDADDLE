package Simulator.Draw;

import Util.Constants;
import Util.Vector;

import java.awt.*;
import javax.swing.*;
import java.util.ArrayList;

/**
 * Provides a graphical interface for both the field simulation window
 * and the pose/trajectory graph window.
 * 
 * This class manages Swing components for robot visualization,
 * spline drawing, interactive point editing, and telemetry charting.
 */
public class Graph extends JPanel {
    public Graph() {
        openFieldWindow();
        openGraphWindow();
        System.out.println("Draw initialized, window opened.");
        moveRobot(0, 0, 0);
    }

    // Field rendering
    public Field field;

    /**
     * Opens the main field visualization window, displaying the robot and paths.
     */
    private void openFieldWindow() {
        String imagePath = "SKIDADDLE/Simulator/Draw/field.png";
        ImageIcon imageIcon = new ImageIcon(imagePath);

        if (imageIcon.getIconWidth() == -1) {
            System.err.println("Error: Image not found at " + imagePath);
            return;
        }

        int width = Constants.FIELD_WIDTH, height = Constants.FIELD_HEIGHT;
        Image scaledImage = imageIcon.getImage().getScaledInstance(width, height, Image.SCALE_SMOOTH);

        field = new Field(scaledImage);
        field.setPreferredSize(new Dimension(width, height));

        JFrame frame = new JFrame("Simulator");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(field, BorderLayout.CENTER);
        frame.pack();
        frame.setLocation(Constants.FIELD_OPEN_X, Constants.FIELD_OPEN_Y);
        frame.setVisible(true);

        // Request focus so that key events work immediately
        SwingUtilities.invokeLater(() -> field.requestFocusInWindow());
    }

    /**
     * Ensures UI updates occur on the Swing event dispatch thread.
     */
    private void onEDT(Runnable r) {
        if (SwingUtilities.isEventDispatchThread()) {
            r.run();
        } else {
            SwingUtilities.invokeLater(r);
        }
    }

    // Robot rendering
    public void moveRobot(double x, double y, double orientation) {
        if (field != null) {
            onEDT(() -> field.moveRobot(x, y, orientation));
        }
    }

    public void setShowRobot(boolean showRobot) {
        if (field != null) {
            field.showRobot = showRobot;
        }
    }

    // Point rendering
    public void drawPoint(Vector v) {
        if (field != null && v != null) {
            onEDT(() -> field.drawPoint(v));
        }
    }

    public void clearPoints() {
        if (field != null) {
            onEDT(() -> field.clearPoints());
        }
    }

    public void removePoint(Vector v) {
        if (field != null) {
            onEDT(() -> field.removePoint(v));
        }
    }

    public void highlightPoint(Vector v) {
        if (field != null && v != null) {
            onEDT(() -> field.highlightPoint(v));
        }
    }

    public void clearHighlight() {
        if (field != null) {
            onEDT(() -> field.clearHighlight());
        }
    }

    // Spline rendering
    public void createPath(String label, float alpha, Color color, float width) {
        if (field != null) {
            onEDT(() -> field.createPath(label, alpha, color, width));
        }
    }

    public void createPath(ArrayList<Vector> values, String label, float alpha, Color color, float width) {
        if (field != null) {
            onEDT(() -> field.createPath(values, label, alpha, color, width));
        }
    }

    public void addPathData(String label, Vector value) {
        if (field != null) {
            onEDT(() -> field.addPathData(label, value));
        }
    }

    public void removeSpline(String label) {
        if (field != null) {
            onEDT(() -> field.removeSpline(label));
        }
    }

    public void clearSplines() {
        if (field != null) {
            onEDT(() -> field.clearSplines());
        }
    }

    // User input tracking
    public boolean enterPressed() {
        if (field != null) {
            return field.consumeEnterPressed();
        }
        return false;
    }

    public boolean rPressed() {
        if (field != null) return field.consumeRPressed();
        return false;
    }

    public boolean backspacePressed() {
        if (field != null) return field.consumeBackspacePressed();
        return false;
    }

    public Vector clicked() {
        if (field != null) return field.consumeClick();
        return null;
    }

    public void updateField() {
        if (field != null) {
            onEDT(() -> field.update());
        }
    }

    // Chart rendering
    private Chart chart = null;

    /**
     * Opens the graph window for plotting telemetry data over time.
     */
    public void openGraphWindow() {
        chart = new Chart();
        chart.setPreferredSize(new Dimension(Constants.CHART_WIDTH, Constants.CHART_HEIGHT));

        JFrame frame = new JFrame("Pose Graph");
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.getContentPane().add(chart, BorderLayout.CENTER);
        frame.pack();
        frame.setLocation(Constants.FIELD_OPEN_X + Constants.FIELD_WIDTH + 20, Constants.FIELD_OPEN_Y);
        frame.setVisible(true);

        // Request focus so key events work immediately
        if (field != null)
            SwingUtilities.invokeLater(() -> field.requestFocusInWindow());
    }

    public void createChart(String label, float alpha, Color color, float strokeWidth) {
        if (chart != null) {
            onEDT(() -> chart.createChart(label, alpha, color, strokeWidth));
        }
    }

    public void addChartData(String label, double value) {
        if (chart != null) {
            onEDT(() -> chart.addChartData(label, value));
        }
    }

    public void addChartData(String label, double value, float alpha, Color color, float strokeWidth) {
        if (chart != null) {
            onEDT(() -> chart.addChartData(label, value, alpha, color, strokeWidth));
        }
    }

    public void updateGraph() {
        if (chart == null) return; // Graph window must be open first
        SwingUtilities.invokeLater(() -> chart.repaint());
    }
}
