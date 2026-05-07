using System.Collections.ObjectModel;
using System.Windows;
using static ManifoldSolver.Core.Analyser;

namespace ManifoldSolver.Core.View
{
    public partial class DataPage : System.Windows.Controls.UserControl
    {
        ObservableCollection<RowDisplay> rows = new ObservableCollection<RowDisplay>();

        int id = 0;

        public DataPage()
        {
            InitializeComponent();

            PipeData.ItemsSource = rows;
        }

        public void AddRow(PipeDef p)
        {
            id++;
            rows.Add(RowDisplay.FromPipeDef(id, p));
        }

        public void Clear()
        {
            rows.Clear();
        }


        public void DoPreanalyse() { }

        private void BuildObstacles_Click(object sender, RoutedEventArgs e) { return; }
        private void PreAnalyse_Click(object sender, RoutedEventArgs e) => DoPreanalyse();
    }

    public class RowDisplay
    {
        public int Id { get; set; }
        public string Name { get; set; } = "";
        public string StartP { get; set; } = "";
        public string EndP { get; set; } = "";
        public string StartN { get; set; } = "";
        public string EndN { get; set; } = "";

        public static RowDisplay FromPipeDef(int id, PipeDef Pipe)
        {
            return new RowDisplay()
            {
                Id = id,
                Name = Pipe.Name,
                StartP = Pipe.Start.ToString(),
                StartN = Pipe.StartDir.ToString(),
                EndP = Pipe.End.ToString(),
                EndN = Pipe.EndDir.ToString()
            };
        }
    }
}
