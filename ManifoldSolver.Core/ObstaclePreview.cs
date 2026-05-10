using GeometrySolver.Conditions;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace ManifoldSolver.Core
{
    internal static class ObstaclePreview
    {
        // Orange (R=255, G=165, B=0) in SolidWorks' BGR-int format
        private const int PreviewColor = 255 + (165 << 8);

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

        public static Body2? PreviewIgnoreSphere(IgnoreSphere sphere, ISldWorks swApp, object component)
        {
            // IModeler has no direct sphere primitive and revolve requires FeatureManager
            // which is unavailable for temp bodies. Sphere preview not supported.
            return null;
        }

        private static void DisplayTransparent(Body2? body, object component)
        {
            if (body == null) return;

            body.Display3(component, PreviewColor, (int)swTempBodySelectOptions_e.swTempBodySelectOptionNone);

            // MaterialPropertyValues2: [R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission]
            // all values 0–1; Transparency 0=opaque, 1=fully transparent
            body.MaterialPropertyValues2 = new double[]
            {
                1.0,          // R
                165.0 / 255.0, // G
                0.0,          // B
                0.4,          // Ambient
                0.8,          // Diffuse
                0.3,          // Specular
                0.3,          // Shininess
                0.6,          // Transparency (60%)
                0.0           // Emission
            };
        }
    }
}
