using System;
using System.Numerics;
using GeometrySolver.Conditions;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace ManifoldSolver.Core
{
    internal static class ObstaclePreview
    {
        // Orange (R=255, G=165, B=0) in SolidWorks' BGR-int format
        private const int PreviewColor = 255 + (165 << 8);
        // Green (R=0, G=200, B=80) in SolidWorks' BGR-int format
        private const int AntiPreviewColor = (200 << 8) + (80 << 16);

        public static Body2? PreviewIgnoreRect(IgnoreRect rect, ISldWorks swApp, object component)
        {
            var modeler = (IModeler)swApp.GetModeler();

            // BoxDimArray: [faceCentre x/y/z, axisDir x/y/z, width, length, height]
            // faceCentre = centre of the -Z face; axis points +Z into the box
            double[] boxDimArray =
            {
                (rect.Min.X + rect.Max.X) * 0.5 / 1000.0,
                (rect.Min.Y + rect.Max.Y) * 0.5 / 1000.0,
                rect.Min.Z / 1000.0,
                0, 0, 1,
                (rect.Max.X - rect.Min.X) / 1000.0,
                (rect.Max.Y - rect.Min.Y) / 1000.0,
                (rect.Max.Z - rect.Min.Z) / 1000.0
            };

            var body = modeler.CreateBodyFromBox3(boxDimArray) as Body2;
            DisplayTransparent(body, component);
            return body;
        }

        public static Body2? PreviewIgnoreCylinder(IgnoreCylinder cyl, ISldWorks swApp, object component)
        {
            var modeler = (IModeler)swApp.GetModeler();

            // CylDimArray: [cylFaceCenter x/y/z, cylAxis x/y/z, cylRadius, cylHeight]
            // cylFaceCenter = centre of one end face; cyl.Centre is the midpoint so shift by -Height/2
            var faceCentreMm = cyl.Centre - cyl.Axis * (cyl.Height * 0.5f);
            double[] cylDimArray =
            {
                faceCentreMm.X / 1000.0, faceCentreMm.Y / 1000.0, faceCentreMm.Z / 1000.0,
                cyl.Axis.X, cyl.Axis.Y, cyl.Axis.Z,
                cyl.Radius / 1000.0,
                cyl.Height / 1000.0
            };

            var body = modeler.CreateBodyFromCyl(cylDimArray) as Body2;
            DisplayTransparent(body, component);
            return body;
        }

        public static Body2? PreviewAntiCylinder(IgnoreCylinder cyl, ISldWorks swApp, object component)
        {
            var modeler = (IModeler)swApp.GetModeler();
            var faceCentreMm = cyl.Centre - cyl.Axis * (cyl.Height * 0.5f);
            double[] cylDimArray =
            {
                faceCentreMm.X / 1000.0, faceCentreMm.Y / 1000.0, faceCentreMm.Z / 1000.0,
                cyl.Axis.X, cyl.Axis.Y, cyl.Axis.Z,
                cyl.Radius / 1000.0,
                cyl.Height / 1000.0
            };
            var body = modeler.CreateBodyFromCyl(cylDimArray) as Body2;
            DisplayAntiTransparent(body, component);
            return body;
        }

        public static Body2? PreviewIgnoreSphere(IgnoreSphere sphere, ISldWorks swApp, object component)
        {
            // IModeler has no direct sphere primitive and revolve requires FeatureManager
            // which is unavailable for temp bodies. Sphere preview not supported.
            return null;
        }

        /// <summary>
        /// Previews a tapered cylinder (frustum) as two concentric transparent cylinders —
        /// one at each radius — so the taper envelope is visible through the transparency.
        /// IModeler has no frustum primitive, so this is the closest available approximation.
        /// </summary>
        public static (Body2? outer, Body2? inner) PreviewIgnoreTaperedCylinder(
            IgnoreTaperedCylinder f, ISldWorks swApp, object component, bool anti = false)
        {
            var modeler = (IModeler)swApp.GetModeler();
            var faceCentreMm = f.Centre - f.Axis * (f.Height * 0.5f);

            Body2? MakeCyl(float radiusMm, Vector3 faceCemtre)
            {
                double[] d =
                {
                    faceCemtre.X / 1000.0, faceCemtre.Y / 1000.0, faceCemtre.Z / 1000.0,
                    f.Axis.X, f.Axis.Y, f.Axis.Z,
                    radiusMm / 1000.0,
                    f.Height / 2000.0
                };
                return modeler.CreateBodyFromCyl(d) as Body2;
            }



            var outer = MakeCyl(Math.Max(f.RadiusBase, f.RadiusTop), f.RadiusBase > f.RadiusTop ? faceCentreMm : f.Centre);
            var inner = MakeCyl(Math.Min(f.RadiusBase, f.RadiusTop), f.RadiusBase < f.RadiusTop ? faceCentreMm : f.Centre);
            if (anti)
            {
                DisplayAntiTransparent(outer, component);
                DisplayAntiTransparent(inner, component);
            }
            else
            {
                DisplayTransparent(outer, component);
                DisplayTransparent(inner, component);
            }
            return (outer, inner);
        }

        private static void DisplayTransparent(Body2? body, object component)
        {
            if (body == null) return;
            body.Display3(component, PreviewColor, (int)swTempBodySelectOptions_e.swTempBodySelectOptionNone);
            body.MaterialPropertyValues2 = new double[]
            {
                1.0,           // R
                165.0 / 255.0, // G
                0.0,           // B
                0.4, 0.8, 0.3, 0.3, 0.6, 0.0
            };
        }

        private static void DisplayAntiTransparent(Body2? body, object component)
        {
            if (body == null) return;
            body.Display3(component, AntiPreviewColor, (int)swTempBodySelectOptions_e.swTempBodySelectOptionNone);
            body.MaterialPropertyValues2 = new double[]
            {
                0.0,           // R
                200.0 / 255.0, // G
                80.0 / 255.0,  // B
                0.4, 0.8, 0.3, 0.3, 0.6, 0.0
            };
        }
    }
}
