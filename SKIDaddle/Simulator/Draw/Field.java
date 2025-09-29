package Simulator.Draw;

import Util.Constants;
import Util.Vector;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.swing.AbstractAction;
import javax.swing.JComponent;
import javax.swing.JPanel;
import javax.swing.KeyStroke;
import javax.swing.SwingUtilities;

/**
 * Field panel for visualizing a robot, waypoints, and spline paths on a 144x144 field.
 * <p>
 * Supports mouse clicks for adding points, key presses for actions,
 * and rendering of the robot, highlighted points, and multiple spline series.
 */
class Field extends JPanel {
    /** Background image of the field. */
    private final Image fieldImage;

    /** Robot position and heading (radians). */
    private double objX, objY, orientation;
    protected boolean showRobot = true;

    /** List of points to render and optional highlighted point. */
    private final java.util.List<Vector> points = new ArrayList<>();
    private Vector highlighted = null;

    /** Key and mouse state flags. */
    private boolean enterPressed = false;
    private boolean rPressed = false;
    private boolean backspacePressed = false;
    private Vector lastClick = null;

    /**
     * Represents a single spline/path series with configurable style and visibility.
     */
    private static class FieldSeries {
        String label;                 // identifier for series
        float alpha;                  // transparency 0.0 → 1.0
        Color color;                  // draw color
        float strokeWidth;            // line thickness
        ArrayList<Vector> values = new ArrayList<>(); // sequence of points
        boolean visible = true;       // toggled rendering

        FieldSeries(String label, float alpha, Color color, float strokeWidth) {
            this.label = label;
            this.alpha = alpha;
            this.color = color;
            this.strokeWidth = strokeWidth;
        }
    }

    /** Map of path labels to their spline series. */
    private Map<String, FieldSeries> seriesMap = new HashMap<>();

    /**
     * Constructs a new field visualization with key/mouse bindings.
     *
     * @param fieldImage background field image
     */
    protected Field(Image fieldImage) {
        this.fieldImage = fieldImage;

        setFocusable(true);
        setFocusTraversalKeysEnabled(false);

        // Key bindings
        getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("ENTER"), "enterPressed");
        getActionMap().put("enterPressed", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                enterPressed = true;
            }
        });

        getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("R"), "rPressed");
        getActionMap().put("rPressed", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                rPressed = true;
            }
        });

        getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("BACK_SPACE"), "backspacePressed");
        getActionMap().put("backspacePressed", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                backspacePressed = true;
            }
        });

        // Mouse click handler
        addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (!Field.this.isFocusOwner())
                    SwingUtilities.invokeLater(() -> requestFocusInWindow());

                double scaleX = getWidth() / 144.0;
                double scaleY = getHeight() / 144.0;

                double fieldX = (e.getX() - getWidth() / 2.0) / scaleX;
                double fieldY = (getHeight() / 2.0 - e.getY()) / scaleY;

                lastClick = new Vector(fieldX, fieldY);
            }
        });

        // Request focus automatically when shown
        addHierarchyListener(e -> {
            if ((e.getChangeFlags() & HierarchyEvent.SHOWING_CHANGED) != 0 && isShowing()) {
                SwingUtilities.invokeLater(this::requestFocusInWindow);
            }
        });
    }

    /**
     * Update the robot’s on-field pose.
     *
     * @param x           X position (field units)
     * @param y           Y position (field units)
     * @param orientation heading in radians (0 = facing up field)
     */
    protected void moveRobot(double x, double y, double orientation) {
        this.objX = x;
        this.objY = y;
        this.orientation = orientation;
    }

    /** Add a point to be drawn on the field. */
    protected void drawPoint(Vector v) {
        points.add(v);
    }

    /** Clear all points and remove highlight. */
    protected void clearPoints() {
        points.clear();
        highlighted = null;
    }

    /**
     * Remove a specific point (with coordinate tolerance).
     *
     * @param v point to remove
     */
    protected void removePoint(Vector v) {
        points.removeIf(p -> coordsEqual(p, v));
        if (highlighted != null && coordsEqual(highlighted, v)) {
            highlighted = null;
        }
    }

    /** Highlight a given point. */
    protected void highlightPoint(Vector v) {
        highlighted = v;
    }

    /** Remove highlight. */
    protected void clearHighlight() {
        highlighted = null;
    }

    /**
     * Create an empty path with the given style.
     * Replaces any existing series with the same label.
     */
    protected void createPath(String label, float alpha, Color color, float strokeWidth) {
        seriesMap.remove(label);

        // Ensure no duplicate colors (nudges brightness if needed)
        Color finalColor = color;
        boolean duplicate = seriesMap.values().stream().anyMatch(s -> s.color.equals(color));
        if (duplicate) {
            int r = Math.min(255, color.getRed() + 90);
            int g = Math.min(255, color.getGreen() + 90);
            int b = Math.min(255, color.getBlue() + 90);
            finalColor = new Color(r, g, b, color.getAlpha());
        }

        seriesMap.put(label, new FieldSeries(label, alpha, finalColor, strokeWidth));
    }

    /** Create a path and directly assign values. */
    protected void createPath(ArrayList<Vector> values, String label, float alpha, Color color, float strokeWidth) {
        createPath(label, alpha, color, strokeWidth);
        seriesMap.get(label).values = values;
    }

    /** Add a new vertex to an existing path. */
    protected void addPathData(String label, Vector value) {
        FieldSeries s = seriesMap.get(label);
        if (s == null) throw new IllegalArgumentException("No path with label '" + label + "'");
        s.values.add(value);
    }

    /** Remove a spline by label. */
    protected void removeSpline(String label) {
        seriesMap.remove(label);
    }

    /** Clear all spline paths. */
    protected void clearSplines() {
        seriesMap.clear();
    }

    /** @return whether Enter was pressed since last call */
    protected boolean consumeEnterPressed() {
        boolean wasPressed = enterPressed;
        enterPressed = false;
        return wasPressed;
    }

    /** @return whether R was pressed since last call */
    protected boolean consumeRPressed() {
        boolean wasPressed = rPressed;
        rPressed = false;
        return wasPressed;
    }

    /** @return whether Backspace was pressed since last call */
    protected boolean consumeBackspacePressed() {
        boolean wasPressed = backspacePressed;
        backspacePressed = false;
        return wasPressed;
    }

    /** @return last click position (field coords), or null if none */
    protected Vector consumeClick() {
        Vector click = lastClick;
        lastClick = null;
        return click;
    }

    /** Trigger a repaint of the field. */
    protected void update() {
        repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        g2.drawImage(fieldImage, 0, 0, getWidth(), getHeight(), this);

        double scaleX = getWidth() / 144.0;
        double scaleY = getHeight() / 144.0;

        int centerX = (int) (getWidth() / 2 + objX * scaleX);
        int centerY = (int) (getHeight() / 2 - objY * scaleY);
        int radius = (int) (9 * Math.min(scaleX, scaleY));

        if (showRobot) {
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(3f));
            int lineX = centerX + (int) (radius * Math.sin(orientation));
            int lineY = centerY - (int) (radius * Math.cos(orientation));
            g2.drawLine(centerX, centerY, lineX, lineY);

            g2.setColor(new Color(0, 200, 180));
            g2.drawOval(centerX - radius, centerY - radius, radius * 2, radius * 2);

            g2.setColor(Color.BLACK);
            g2.fillOval(centerX - 4, centerY - 4, 8, 8);
        }

        g2.setStroke(new BasicStroke(1.5f));
        int pointRadius = 6;
        for (Vector v : points) {
            int px = (int) (getWidth() / 2 + v.x * scaleX);
            int py = (int) (getHeight() / 2 - v.y * scaleY);

            g2.setColor(Color.WHITE);
            g2.fillOval(px - pointRadius, py - pointRadius, pointRadius * 2, pointRadius * 2);
            g2.setColor(Color.BLACK);
            g2.drawOval(px - pointRadius, py - pointRadius, pointRadius * 2, pointRadius * 2);

            if (coordsEqual(v, highlighted)) {
                g2.setColor(Color.YELLOW);
                int hlRadius = pointRadius + 4;
                g2.setStroke(new BasicStroke(3f));
                g2.drawOval(px - hlRadius, py - hlRadius, hlRadius * 2, hlRadius * 2);
            }
        }

        for (FieldSeries ps : seriesMap.values()) {
            Color drawColor = new Color(ps.color.getRed(), ps.color.getGreen(), ps.color.getBlue(),
                    (int) (ps.alpha * 255));
            g2.setColor(drawColor);
            g2.setStroke(new BasicStroke(ps.strokeWidth));
            for (int i = 1; i < ps.values.size(); i++) {
                Vector v1 = ps.values.get(i - 1);
                Vector v2 = ps.values.get(i);
                int x1 = (int) (getWidth() / 2 + v1.x * scaleX);
                int y1 = (int) (getHeight() / 2 - v1.y * scaleY);
                int x2 = (int) (getWidth() / 2 + v2.x * scaleX);
                int y2 = (int) (getHeight() / 2 - v2.y * scaleY);
                g2.drawLine(x1, y1, x2, y2);
            }
        }
    }

    /**
     * Compare two vectors for equality with a small tolerance.
     */
    private static boolean coordsEqual(Vector a, Vector b) {
        if (a == null || b == null) return false;
        return Math.abs(a.x - b.x) < Constants.ZERO_TOLERANCE
            && Math.abs(a.y - b.y) < Constants.ZERO_TOLERANCE;
    }
}
