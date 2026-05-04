using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;

namespace ManifoldSolver.AddIn
{
    /// <summary>
    /// Generates isogrid-pattern icon PNG files at runtime and returns their file paths.
    ///
    /// SolidWorks reads icon images from absolute file paths set on ICommandGroup before
    /// Activate() is called. Icons are written once to %TEMP%\IsogridGenerator\Icons\ and
    /// reused on subsequent loads.
    ///
    /// The icon shows a 2-row equilateral triangular grid (6 triangles) — a minimal but
    /// immediately recognisable representation of an isogrid pattern.
    /// </summary>
    internal static class IconHelper
    {
        private static string? _largeIconPath;
        private static string? _smallIconPath;

        // ── Public API ────────────────────────────────────────────────────────

        /// <summary>
        /// Ensures the 24×24 (large) and 16×16 (small) icon PNGs exist on disk.
        /// Returns their absolute paths in a tuple; both values are guaranteed non-null
        /// on success.  Returns (null, null) if icon generation fails — callers should
        /// tolerate missing icons gracefully.
        /// </summary>
        public static (string? Large, string? Small) EnsureIcons()
        {
            if (_largeIconPath != null && _smallIconPath != null)
                return (_largeIconPath, _smallIconPath);

            try
            {
                var dir = Path.Combine(Path.GetTempPath(), "IsogridGenerator", "Icons");
                Directory.CreateDirectory(dir);

                _largeIconPath = Path.Combine(dir, "isogrid_24.png");
                _smallIconPath = Path.Combine(dir, "isogrid_16.png");

                WriteIsogridPng(24, _largeIconPath);
                WriteIsogridPng(16, _smallIconPath);

                return (_largeIconPath, _smallIconPath);
            }
            catch
            {
                // Icon generation is best-effort; a missing icon must not prevent
                // the add-in from loading.
                _largeIconPath = null;
                _smallIconPath = null;
                return (null, null);
            }
        }

        // ── Icon drawing ──────────────────────────────────────────────────────

        private static void WriteIsogridPng(int size, string path)
        {
            using var bmp = new Bitmap(size, size, PixelFormat.Format32bppArgb);
            using var g   = Graphics.FromImage(bmp);

            g.Clear(Color.White);
            g.SmoothingMode = SmoothingMode.AntiAlias;

            DrawIsogridPattern(g, size);
            bmp.Save(path, ImageFormat.Png);
        }

        /// <summary>
        /// Draws a 2-row equilateral triangle grid that fills the given pixel square.
        ///
        /// Vertex layout (A = top row, B = mid row, C = bottom row):
        ///
        ///   A0 ——— A1 ——— A2
        ///    \  ▽  /  △  \  ▽  /
        ///     B0 ——————— B1
        ///    /  △  \  ▽  /  △  \
        ///   C0 ——— C1 ——— C2
        ///
        /// Six triangles in total — three pointing down (▽) and three pointing up (△).
        /// </summary>
        private static void DrawIsogridPattern(Graphics g, int size)
        {
            // Triangle edge length: leave 2 px padding on each side; 2 columns of edge
            // width fit across the drawing area (3 vertices per row, 2 gaps = 2a).
            const float pad = 2f;
            float a = (size - 2f * pad) / 2f;   // edge length
            float h = a * 0.8660254f;             // row height = a * √3/2

            float xPad = pad;
            float yPad = (size - 2f * h) / 2f;   // vertical centering

            // ── Vertices ──────────────────────────────────────────────────────
            PointF A0 = new PointF(xPad,              yPad);
            PointF A1 = new PointF(xPad + a,          yPad);
            PointF A2 = new PointF(xPad + 2f * a,     yPad);

            PointF B0 = new PointF(xPad + a / 2f,     yPad + h);
            PointF B1 = new PointF(xPad + 3f * a / 2f, yPad + h);

            PointF C0 = new PointF(xPad,              yPad + 2f * h);
            PointF C1 = new PointF(xPad + a,          yPad + 2f * h);
            PointF C2 = new PointF(xPad + 2f * a,     yPad + 2f * h);

            // ── Draw ──────────────────────────────────────────────────────────
            float penWidth = Math.Max(0.8f, size / 20f);

            using var pen = new Pen(Color.FromArgb(255, 30, 100, 200), penWidth)
            {
                StartCap = LineCap.Round,
                EndCap   = LineCap.Round,
            };

            // All 13 unique edges of the 6-triangle grid
            (PointF From, PointF To)[] edges =
            {
                // Horizontal rows
                (A0, A1), (A1, A2),
                (B0, B1),
                (C0, C1), (C1, C2),

                // Upper diagonals (A → B)
                (A0, B0), (A1, B0),
                (A1, B1), (A2, B1),

                // Lower diagonals (B → C)
                (B0, C0), (B0, C1),
                (B1, C1), (B1, C2),
            };

            foreach (var (from, to) in edges)
                g.DrawLine(pen, from, to);
        }
    }
}
