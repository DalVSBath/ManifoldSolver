using CADBooster.SolidDna;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace ManifoldSolver.AddIn
{
    // Replace this GUID with a fresh one before shipping.
    // Generate one in PowerShell:  [System.Guid]::NewGuid().ToString()
    [Guid("a6a68281-d61d-4575-8623-7e035289e74e")]
    [ComVisible(true)]
    public class ManifoldSolverAddIn : SolidAddIn
    {

        private CommandSetup? _commands;

        // SolidDNA 4.0: replaces the old ApplicationStarted() override.
        public override void ApplicationStartup()
        {
            _commands = new CommandSetup(this);
            _commands.Register();
        }

        // SolidDNA 4.0: called before the SW connection is established.
        // Register any plug-ins here if needed; leave empty otherwise.
        public override void PreConnectToSolidWorks() { }

        // SolidDNA 4.0: called before plug-ins are loaded.
        // Add plug-in types via PlugInIntegration.AddPlugInToLoad<T>() if needed.
        public override void PreLoadPlugIns() { }

        // Callback invoked by SolidWorks when the Isogrid toolbar button is clicked.
        // The method name is passed as a string in AddCommandItem2 — it must be public.
        public void OnIsogridClick()
        {
            _commands?.HandleIsogridClick();
        }
    }
}
