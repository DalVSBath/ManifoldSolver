using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Conditions
{
    /// <summary>
    /// Classifies the type of solver condition for reporting and dispatch purposes.
    /// </summary>
    public enum ConditionType
    {
        ObstacleSphere,
        ObstacleBox,
        ObstacleCylinder,
        PipeClearance,
        MinimumStraight,
        MaxBendAngle,
        Custom,
    }

    /// <summary>
    /// The extensible constraint interface for all solver conditions.
    ///
    /// Every constraint (obstacle, clearance, geometric rule, custom) implements this
    /// two-method contract:
    ///   - <see cref="IsSatisfied"/> is the hard pass/fail gate evaluated in
    ///     <c>BuildSolution</c> after a candidate path has been constructed.
    ///   - <see cref="Penalty"/> returns a continuous differentiable cost that is
    ///     added to the optimizer's residual, allowing the Adam solver to steer away
    ///     from violating configurations during grid search and refinement.
    ///
    /// Path geometry is provided as a pre-sampled list of 3-D centreline points (see
    /// <c>PathSampler</c>). Conditions should not re-simulate the path themselves.
    /// </summary>
    public interface ISolverCondition
    {
        /// <summary>Identifies the category of this condition.</summary>
        ConditionType Type { get; }

        /// <summary>
        /// Returns <c>true</c> when the candidate path satisfies the constraint.
        /// A return value of <c>false</c> causes the candidate to be rejected outright.
        /// </summary>
        /// <param name="pathPoints">Evenly-spaced centreline points from PathSampler.</param>
        /// <param name="pipeDiameter">Outer pipe diameter used as the clearance buffer.</param>
        bool IsSatisfied(IReadOnlyList<Vector3> pathPoints, float pipeDiameter);

        /// <summary>
        /// Returns a non-negative penalty value that is zero when the constraint is
        /// satisfied and grows continuously as the violation worsens. Used as an
        /// additive term in the optimizer residual.
        /// </summary>
        /// <param name="pathPoints">Evenly-spaced centreline points from PathSampler.</param>
        /// <param name="pipeDiameter">Outer pipe diameter used as the clearance buffer.</param>
        double Penalty(IReadOnlyList<Vector3> pathPoints, float pipeDiameter);
    }
}
