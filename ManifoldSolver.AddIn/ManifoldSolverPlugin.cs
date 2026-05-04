using CADBooster.SolidDna;
using System;
using System.Windows;

namespace IsogridGenerator.AddIn
{
    public class ManifoldSolverPlugin : SolidPlugIn
    {
        public override string AddInTitle => "Manifold Solver";
        public override string AddInDescription => "Geometry solver for manifold pipe routing";

        public override void ConnectedToSolidWorks()
        {
        }

        public override void DisconnectedFromSolidWorks()
        {

        }
    }
}
