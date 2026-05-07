using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ManifoldSolver.UI.Helpers
{
    internal static class EditPageItems
    {
        public static IPropertyManagerPageSelectionbox AddSelectionBox(this IPropertyManagerPage2? _page,
            int id, string caption, string tip, short indent,
            swSelectType_e[]? selectionFilters = null)
        {
            var selector = (IPropertyManagerPageSelectionbox)_page.AddControl2(
                id,
                (short)swPropertyManagerPageControlType_e.swControlType_Selectionbox,
                "Select face",
                indent,
                (int)swAddControlOptions_e.swControlOptions_Enabled | (int)swAddControlOptions_e.swControlOptions_Visible,
                "Click here, then select a face in the viewport");

            if(selectionFilters is not null)
                selector.SetSelectionFilters(selectionFilters.Select(x => (int)x).ToArray());

            return selector;
        }

        public static IPropertyManagerPageNumberbox AddIntBox(this IPropertyManagerPage2 _page,
            int id, string caption, string tip,
            int min, int max, int? defaultValue = null)
        {
            var box = _page.AddNumberBox(id, caption, tip, min, max, defaultValue);


            box.SetRange2(
                (short)swNumberboxUnitType_e.swNumberBox_UnitlessInteger,
                min, max,
                true,   // Inclusive — clamp to [Min, Max]
                1,    // normal scroll increment (mm)
                5,    // fast scroll increment (mm)
                1);  // slow scroll increment (mm)

            return box;
        }

        public static IPropertyManagerPageNumberbox AddNumberBox(this IPropertyManagerPage2 _page,
            int id, string caption, string tip,
            double min, double max, double? defaultValue = null, swNumberboxUnitType_e unitType = swNumberboxUnitType_e.swNumberBox_UnitlessDouble)
        {
            int enabled = (int)swAddControlOptions_e.swControlOptions_Enabled
                        | (int)swAddControlOptions_e.swControlOptions_Visible;
            short indent = (short)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;

            var box = (IPropertyManagerPageNumberbox)_page!.AddControl2(
                id,
                (short)swPropertyManagerPageControlType_e.swControlType_Numberbox,
                caption,
                indent,
                enabled,
                tip);


            box.SetRange2(
                (short)unitType,
                min, max,
                true,   // Inclusive — clamp to [Min, Max]
                0.1,    // normal scroll increment (mm)
                1.0,    // fast scroll increment (mm)
                0.01);  // slow scroll increment (mm)


            if (defaultValue != null)
                box.Value = defaultValue.Value;

            return box;
        }
    }
}
