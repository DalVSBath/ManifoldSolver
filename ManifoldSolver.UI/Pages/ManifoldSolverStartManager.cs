using CADBooster.SolidDna;
using ManifoldSolver.Core;
using ManifoldSolver.Core.ViewModels;
using ManifoldSolver.UI.Helpers;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Documents;

namespace ManifoldSolver.UI.Pages
{
    public class ManifoldSolverStartManager : IPropertyManagerPage2Handler9
    {
        private const int IdStartPointSelections = 20, IdStartNormalSelections = 21,
            IdEndPointSelections = 30, IdEndNormalSelections = 31,
            IdTargetLengthBox = 40, IdPipeDiameterBox = 41, IdMaxBendsBox = 46, IdMinStraightLengthBox = 42, IdMinPipeToPipeClearanceBox = 43,
            IdLengthToleranceBox = 44, IdMaxBacktrackCandidatesBox = 47, IdMaxBendAngleBox = 45,
            IdPossibleBendRadiiBox = 50;

        private IPropertyManagerPageSelectionbox? _startPointSelection, _startNormalSelection, _endPointSelection, _endNormalSelection;
        private IPropertyManagerPageNumberbox? _targetLengthBox, _pipeDiamterBox, _maxBendsBox, _minStraightBox, _pipeCleranceBox,
            _lengthToleranceBox, _backtrackBox, _bendAngleBox;

        private IPropertyManagerPage2? _page;

        private readonly ISldWorks _swApp;
        private readonly IModelDoc2 _doc;

        private AnalyserViewModel _inputs = new AnalyserViewModel();
        private bool _isClosing = false;

        public ManifoldSolverStartManager(ISldWorks swApp, IModelDoc2 doc)
        {
            _swApp = swApp;
            _doc = doc;
        }

        public void Show()
        {
            int errors = 0;
            // swPropertyManagerOptions_OkButton = 1, swPropertyManagerOptions_CancelButton = 2
            _page = (IPropertyManagerPage2)_swApp.CreatePropertyManagerPage(
                "Isogrid Generator",
                1 | 2,
                this,
                ref errors);

            if (_page == null || errors != 0)
            {
                MessageBox.Show("Failed to create PropertyManagerPage.");
                return;
            }

            BuildControls();
            _page.Show2(0);
        }

        private void BuildControls()
        {
            if (_page == null) return;

            int enabled = (int)swAddControlOptions_e.swControlOptions_Enabled
                        | (int)swAddControlOptions_e.swControlOptions_Visible;
            short indent = (short)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;

            // ── Selection box ────────────────────────────────────────────────────
            _startPointSelection = _page.AddSelectionBox(IdStartPointSelections,
                "Select Start Points", "Click here, then select a face in the viewport",
                indent, new swSelectType_e[] { swSelectType_e.swSelVERTICES, swSelectType_e.swSelPOINTREFS, swSelectType_e.swSelEXTSKETCHPOINTS });
            _startPointSelection.Height = 50;

            _startNormalSelection = _page.AddSelectionBox(IdStartNormalSelections,
                "Select Start Faces", "Select Faces to generate normals for the start points",
                indent, new swSelectType_e[] { swSelectType_e.swSelFACES, swSelectType_e.swSelDATUMPLANES, swSelectType_e.swSelSKETCHPOINTS, swSelectType_e.swSelEXTSKETCHPOINTS, swSelectType_e.swSelDATUMPOINTS });
            _startNormalSelection.Height = 50;

            _endPointSelection = _page.AddSelectionBox(IdEndPointSelections,
                "Select End Points", "Click here, then select a face in the viewport",
                indent, new swSelectType_e[] { swSelectType_e.swSelVERTICES, swSelectType_e.swSelPOINTREFS, swSelectType_e.swSelSKETCHPOINTS, swSelectType_e.swSelEXTSKETCHPOINTS, swSelectType_e.swSelDATUMPOINTS });
            _endPointSelection.Height = 50;

            _endNormalSelection = _page.AddSelectionBox(IdEndNormalSelections,
                "Select End Faces", "Select Faces to generate normals for the end points",
                indent, new swSelectType_e[] { swSelectType_e.swSelFACES, swSelectType_e.swSelDATUMPLANES });
            _endNormalSelection.Height = 50;


            // ── Number boxes (values entered in mm) ──────────────────────────────
            _targetLengthBox = _page.AddNumberBox(IdTargetLengthBox, 
                "Target Runner Length", "Runner length target in mm for the system to solve.", 50, 1500);

            _pipeDiamterBox = _page.AddNumberBox(IdPipeDiameterBox, "Pipe Diameter", "Diameter of exhaust pipe",
                10, 200);

            _minStraightBox = _page.AddNumberBox(IdMinStraightLengthBox, "Min Straight Section", "Min", 0, 500, 0);

            _lengthToleranceBox = _page.AddNumberBox(IdLengthToleranceBox, "Tolerance ± Length", "Tolerance of the overall length of each runner", 0, 100, 10);
            _maxBendsBox = _page.AddNumberBox(IdMaxBendAngleBox, "Max Bend", "Maximum single bend angle to be performed (usually 180)", 90, 360, 180);


            _maxBendsBox = _page.AddIntBox(IdMaxBendsBox, "Max Bends", "Maximum number of bends per pipe (8 is the max supported)", 1, 10, 4);

            _backtrackBox = _page.AddIntBox(IdMaxBacktrackCandidatesBox, "Max Backtracks", "Maximum number of times the system can back track (greatly affects runtime).", 1, 10, 4);

            IdToMark.Clear();

            _startPointSelection.Mark = 1;
            IdToMark.Add(IdStartPointSelections, _startPointSelection.Mark);
            _startNormalSelection.Mark = 2;
            IdToMark.Add(IdStartNormalSelections, _startNormalSelection.Mark);
            _endPointSelection.Mark = 4;
            IdToMark.Add(IdEndPointSelections, _endPointSelection.Mark);
            _endNormalSelection.Mark = 8;
            IdToMark.Add(IdEndNormalSelections, _endNormalSelection.Mark);
        }

        Dictionary<int, int> IdToMark = new Dictionary<int, int>();

        public void AfterActivation()
        {
            
        }

        public void OnClose(int Reason)
        {
            // Mark as closing FIRST so OnSelectionboxSelectionChanged's count==0 callback
            // (fired by SolidWorks as it clears the selection box during close) does not
            // null out _inputs.SelectedFace before HandleOk() reads it.
            _isClosing = true;

            if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Okay)
                HandleOk();

        }

        public void AfterClose()
        {
            
        }

        public bool OnHelp()
        {
            return false;
        }

        public bool OnPreviousPage()
        {
            return false;
        }

        public bool OnNextPage()
        {
            return false;
        }

        public bool OnPreview()
        {
            return false;
        }

        public void OnWhatsNew()
        {
            
        }

        public void OnUndo()
        {
            
        }

        public void OnRedo()
        {
            
        }

        public bool OnTabClicked(int Id)
        {

            return false;
        }

        public void OnGroupExpand(int Id, bool Expanded)
        {
            
        }

        public void OnGroupCheck(int Id, bool Checked)
        {
            
        }

        public void OnCheckboxCheck(int Id, bool Checked)
        {
            
        }

        public void OnOptionCheck(int Id)
        {
            
        }

        public void OnButtonPress(int Id)
        {
            
        }

        public void OnTextboxChanged(int Id, string Text)
        {
            
        }

        public void OnNumberboxChanged(int Id, double value)
        {

            switch (Id)
            {
                case IdTargetLengthBox: _inputs.TargetLength = value; break;
                case IdPipeDiameterBox: _inputs.PipeDiameter = value; break;
                case IdMinStraightLengthBox: _inputs.MinStraight = value; break;
                case IdMinPipeToPipeClearanceBox: _inputs.Clearance = value; break;
                case IdLengthToleranceBox: _inputs.LengthTolerance = value; break;
                case IdMaxBendAngleBox: _inputs.MaxAngle = value; break;

                case IdMaxBendsBox: _inputs.MaxBends = (int)value; break;
                case IdMaxBacktrackCandidatesBox: _inputs.MaxBacktrack = (int)value; break;
            }
        }

        public void OnComboboxEditChanged(int Id, string Text)
        {
            
        }

        public void OnComboboxSelectionChanged(int Id, int Item)
        {
            
        }

        public void OnListboxSelectionChanged(int Id, int Item)
        {
            
        }

        public void OnSelectionboxFocusChanged(int Id)
        {
            
        }


        public void OnSelectionboxListChanged(int Id, int Count)
        {
            if (_isClosing) return;

            UpdateSelectionBox(Id, Count);
        }
        public void OnSelectionboxSelectionChanged(int id, int count)
        {
            if (_isClosing) return;
            UpdateSelectionBox(id, count);
        }


        public void OnSelectionboxCalloutCreated(int Id)
        {
            
        }

        public void OnSelectionboxCalloutDestroyed(int Id)
        {
            
        }

        public bool OnSubmitSelection(int Id, object Selection, int SelType, ref string ItemText)
        {
            return true;
        }

        public int OnActiveXControlCreated(int Id, bool Status)
        {
            return 0;
        }

        public void OnSliderPositionChanged(int Id, double Value)
        {
            
        }

        public void OnSliderTrackingCompleted(int Id, double Value)
        {
            
        }

        public bool OnKeystroke(int Wparam, int Message, int Lparam, int Id)
        {
            return false;
        }

        public void OnPopupMenuItem(int Id)
        {
            
        }

        public void OnPopupMenuItemUpdate(int Id, ref int retval)
        {
            
        }

        public void OnGainedFocus(int Id)
        {
            
        }

        public void OnLostFocus(int Id)
        {
            
        }

        public int OnWindowFromHandleControlCreated(int Id, bool Status)
        {
            return 0;
        }

        public void OnListboxRMBUp(int Id, int PosX, int PosY)
        {
            
        }

        public void OnNumberBoxTrackingCompleted(int Id, double Value)
        {
            
        }



        #region Helpers
        private void HandleOk()
        {
            SyncInputsFromBoxes();

            new Analyser().RunAnalysis(_swApp, _inputs);
        }

        private void SyncInputsFromBoxes()
        {
            if (_targetLengthBox != null) _inputs.TargetLength = _targetLengthBox.Value;
            if (_pipeDiamterBox != null) _inputs.PipeDiameter = _pipeDiamterBox.Value;
            if (_minStraightBox != null) _inputs.MinStraight = _minStraightBox.Value;
            if (_pipeCleranceBox != null) _inputs.Clearance = _pipeCleranceBox.Value;
            if (_lengthToleranceBox != null) _inputs.LengthTolerance = _lengthToleranceBox.Value;
            if (_bendAngleBox != null) _inputs.MaxAngle = _bendAngleBox.Value;

            if (_maxBendsBox != null) _inputs.MaxBends = (int)_maxBendsBox.Value;
            if (_backtrackBox != null) _inputs.MaxBacktrack = (int)_backtrackBox.Value;
        }

        public void UpdateSelectionBox(int Id, int Count)
        {
            switch(Id)
            {
                case IdEndNormalSelections:
                case IdStartNormalSelections:
                    GetNormalsFromSelection(Id, Count); break;

                case IdStartPointSelections:
                case IdEndPointSelections:
                    GetPointsFromSelection(Id, Count); break;
            }
        }


        public void GetNormalsFromSelection(int Id, int Count)
        {
            int mark = IdToMark[Id];
            if (Count > 0)
            {
                var selMgr = (ISelectionMgr)_doc.SelectionManager;
                int total = selMgr.GetSelectedObjectCount2(mark);

                List<Vector3> normals = new List<Vector3>();

                for (int i = 1; i <= total; i++)
                {
                    var obj = selMgr.GetSelectedObject6(i, mark);
                    if (obj is IFace2 face)
                    {
                        var norm = (double[])face.Normal;

                        normals.Add(new Vector3((float)norm[0], (float)norm[1], (float)norm[2]));

                    }else if (obj is IFeature plane)
                    {
                        if (plane.GetSpecificFeature2() is IRefPlane refPlane)
                        {

                            MathTransform transform = refPlane.Transform; 
                            Component2 component = (Component2)selMgr.GetSelectedObjectsComponent4(i, mark);

                            MathTransform finalXform = transform;
                            if (component != null)
                            {
                                MathTransform compXform = component.Transform2;
                                finalXform = (MathTransform)transform.Multiply(compXform);
                            }

                            double[] norm = (double[])finalXform.ArrayData;
                            normals.Add(new Vector3((float)norm[6], (float)norm[7], (float)norm[8]));
                        }
                    }
                }

                switch (Id)
                {
                    case IdStartNormalSelections:
                        _inputs.StartNormals = normals.ToArray(); break;
                    case IdEndNormalSelections:
                        _inputs.EndNormals = normals.ToArray(); break;
                }
            }
            else if (!_isClosing)
            {
                switch (Id)
                {
                    case IdStartNormalSelections:
                        _inputs.StartNormals = null; break;
                    case IdEndNormalSelections:
                        _inputs.EndNormals = null; break;
                }
            }
        }

        public void GetPointsFromSelection(int Id, int Count)
        {
            int mark = IdToMark[Id];
            if (Count > 0)
            {
                var selMgr = (ISelectionMgr)_doc.SelectionManager;
                int total = selMgr.GetSelectedObjectCount2(mark);

                List<Vector3> points = new List<Vector3>();

                for (int i = 1; i <= total; i++)
                {
                    double[] localCoords;
                    var obj = selMgr.GetSelectedObject6(i, mark);
                    if (obj is ISketchPoint sketchPoint)
                    {
                        localCoords = new[] { sketchPoint.X, sketchPoint.Y, sketchPoint.Z };
                    }
                    else if(obj is Vertex vert)
                    {
                        localCoords = (double[]) vert.GetPoint();
                    }else
                    {
                        continue;
                    }

                    Component2 component = (Component2)selMgr.GetSelectedObjectsComponent4(i, mark);

                    double[] worldCoords;
                    if (component != null)
                    {
                        // Use MathUtility to transform the point properly
                        var mathUtility = (MathUtility)_swApp.GetMathUtility();
                        var localPoint = (MathPoint)mathUtility.CreatePoint(localCoords);
                        var worldPoint = (MathPoint)localPoint.MultiplyTransform(component.Transform2);
                        worldCoords = (double[])worldPoint.ArrayData;
                    }
                    else
                    {
                        worldCoords = localCoords;
                    }

                    points.Add(new Vector3((float)worldCoords[0], (float)worldCoords[1], (float)worldCoords[2]));
                }
            }
        }
        #endregion
    }
}
