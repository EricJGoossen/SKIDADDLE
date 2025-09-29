package Simulator.Draw;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import javax.swing.JPanel;

/**
 * A panel that draws line charts for multiple labeled data series.
 * Each series can be toggled on or off by clicking its legend entry.
 * Supports hover tooltips that show the values at the nearest index.
 */
class Chart extends JPanel implements MouseMotionListener {
    private int nearestIndex = -1;

    /**
     * Represents a single charted data series with styling and values.
     */
    private static class ChartSeries {
        String label;
        float alpha;
        Color color;
        float strokeWidth;
        java.util.List<Double> values = new ArrayList<>();
        boolean visible = true;

        ChartSeries(String label, float alpha, Color color, float strokeWidth) {
            this.label = label;
            this.alpha = alpha;
            this.color = color;
            this.strokeWidth = strokeWidth;
        }
    }

    private final Map<String, ChartSeries> seriesMap = new LinkedHashMap<>();

    /**
     * Constructs a Chart panel and sets up mouse listeners for
     * hovering and toggling series visibility.
     */
    protected Chart() {
        addMouseMotionListener(this);

        addMouseListener(new java.awt.event.MouseAdapter() {
            @Override public void mouseClicked(MouseEvent e) {
                int lx = getWidth() - 160, ly = 40, lh = 20;
                int i = 0;
                for (ChartSeries s : seriesMap.values()) {
                    Rectangle rect = new Rectangle(lx, ly + i * lh - 12, 120, lh);
                    if (rect.contains(e.getPoint())) {
                        s.visible = !s.visible;
                        repaint();
                    }
                    i++;
                }
            }
        });
    }

    /**
     * Creates a new chart series or replaces an existing one.
     * If the color is already used, a brighter variant is applied.
     *
     * @param label unique series name
     * @param alpha transparency factor (0–1)
     * @param color base color
     * @param strokeWidth line thickness
     */
    protected void createChart(String label, float alpha, Color color, float strokeWidth) {
        seriesMap.remove(label);

        Color finalColor = color;

        boolean duplicate = seriesMap.values().stream()
                .anyMatch(s -> s.color.equals(color));

        if (duplicate) {
            int r = Math.min(255, color.getRed() + 90);
            int g = Math.min(255, color.getGreen() + 90);
            int b = Math.min(255, color.getBlue() + 90);
            finalColor = new Color(r, g, b, color.getAlpha());
        }

        seriesMap.put(label, new ChartSeries(label, alpha, finalColor, strokeWidth));
    }

    /**
     * Appends a data point to an existing chart series.
     *
     * @param label series name
     * @param value numeric value to append
     */
    protected void addChartData(String label, double value) {
        ChartSeries s = seriesMap.get(label);
        if (s == null) throw new IllegalArgumentException("No chart with label '" + label + "'");
        s.values.add(value);
    }

    /**
     * Appends data to a series, creating the series if it does not exist.
     *
     * @param label series name
     * @param value numeric value
     * @param alpha transparency factor
     * @param color line color
     * @param strokeWidth line thickness
     */
    protected void addChartData(String label, double value, float alpha, Color color, float strokeWidth) {
        ChartSeries s = seriesMap.get(label);
        if (s == null) {
            s = new ChartSeries(label, alpha, color, strokeWidth);
            seriesMap.put(label, s);
        }
        s.values.add(value);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        if (seriesMap.isEmpty()) return;

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int width = getWidth() - 220;
        int height = getHeight() - 80;
        int originX = 60, topMargin = 20;

        // find global min and max among all visible series
        double minVal = Double.POSITIVE_INFINITY;
        double maxVal = Double.NEGATIVE_INFINITY;
        for (ChartSeries s : seriesMap.values()) {
            if (!s.visible || s.values.isEmpty()) continue;
            minVal = Math.min(minVal, s.values.stream().mapToDouble(Double::doubleValue).min().orElse(0));
            maxVal = Math.max(maxVal, s.values.stream().mapToDouble(Double::doubleValue).max().orElse(0));
        }
        if (minVal == Double.POSITIVE_INFINITY || maxVal == Double.NEGATIVE_INFINITY) return;

        double range = maxVal - minVal;
        if (range == 0) range = 1;

        int originY = topMargin + (int)((maxVal / range) * (height - topMargin));

        // axes
        g2.setColor(Color.BLACK);
        g2.drawLine(originX, topMargin, originX, height);  // y-axis
        g2.drawLine(originX, originY, originX + width, originY); // x-axis

        // draw series
        for (ChartSeries s : seriesMap.values()) {
            if (!s.visible || s.values.size() < 2) continue;
            g2.setStroke(new BasicStroke(s.strokeWidth));
            g2.setColor(new Color(s.color.getRed(), s.color.getGreen(), s.color.getBlue(), (int)(s.alpha * 255)));

            for (int i = 1; i < s.values.size(); i++) {
                int x1 = originX + (i - 1) * width / (s.values.size() - 1);
                int y1 = scaleY(s.values.get(i - 1), minVal, maxVal, height, topMargin);
                int x2 = originX + i * width / (s.values.size() - 1);
                int y2 = scaleY(s.values.get(i), minVal, maxVal, height, topMargin);
                g2.drawLine(x1, y1, x2, y2);
            }
        }

        // legend
        g2.setFont(new Font("Arial", Font.PLAIN, 14));
        int legendX = originX + width + 40, legendY = topMargin + 20, lineH = 20;
        int i = 0;
        for (ChartSeries s : seriesMap.values()) {
            g2.setColor(s.color);
            g2.fillRect(legendX, legendY + i * lineH - 12, 15, 15);
            g2.setColor(Color.BLACK);
            g2.drawString(s.label + (s.visible ? "" : " (off)"),
                    legendX + 20, legendY + i * lineH);
            i++;
        }

        // hover tooltip
        int maxLen = getMaxSeriesLength();
        if (nearestIndex >= 0 && maxLen > 1) {
            int hoverX = originX + nearestIndex * width / (maxLen - 1);
            g2.setColor(new Color(0, 0, 0, 100));
            g2.drawLine(hoverX, topMargin, hoverX, height);

            long visibleCount = seriesMap.values().stream()
                    .filter(s -> s.visible && nearestIndex < s.values.size())
                    .count();

            int boxX = legendX, boxY = legendY + seriesMap.size() * lineH + 30;
            int boxHeight = 40 + (int)visibleCount * 18;
            g2.setColor(new Color(255, 255, 230));
            g2.fillRect(boxX, boxY, 200, boxHeight);
            g2.setColor(Color.BLACK);
            g2.drawRect(boxX, boxY, 200, boxHeight);

            g2.setFont(new Font("Arial", Font.BOLD, 13));
            int ty = boxY + 20;
            g2.drawString("Index = " + nearestIndex, boxX + 10, ty);
            ty += 20;

            g2.setFont(new Font("Arial", Font.PLAIN, 12));
            for (ChartSeries s : seriesMap.values()) {
                if (!s.visible || nearestIndex >= s.values.size()) continue;
                g2.setColor(Color.BLACK);
                g2.drawString(s.label + ": " + String.format("%.2f", s.values.get(nearestIndex)),
                        boxX + 10, ty);
                ty += 18;
            }
        }
    }

    private int scaleY(double val, double minVal, double maxVal, int height, int topMargin) {
        double range = maxVal - minVal;
        if (range == 0) range = 1;
        return topMargin + (int)((maxVal - val) / range * (height - topMargin));
    }

    private int getMaxSeriesLength() {
        return seriesMap.values().stream().mapToInt(s -> s.values.size()).max().orElse(1);
    }

    @Override public void mouseDragged(MouseEvent e) {}

    @Override public void mouseMoved(MouseEvent e) {
        int maxLen = getMaxSeriesLength();
        if (maxLen < 2) return;

        int width = getWidth() - 220, originX = 60;
        nearestIndex = Math.max(0, Math.min(maxLen - 1,
                (int) ((e.getX() - originX) / (double) width * maxLen)));
        repaint();
    }
}
