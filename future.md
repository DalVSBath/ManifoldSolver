# Future Solver Plans



The plan is for this solver to create equal length exhaust manifolds within a specific tolerance limit. This will need to solve up to 8 pipes (curves such as this) at once. So you'll need to account for that inside your solution. The initial issue I see is that there will be a requirement maybe not in this specific case to have maybe 5/6 bends or more incorporated so we need to have that ability in the future.



The future requirements will also be a lot more controlled and there will need to be consideration for environmental variables, I see this probably more as an optimisation problem than a mathematical problem of testing pairs. 





### All Future Conditions



Just to give a full understanding of all the future conditions that will need to be considered, there may be more than this



#### Internal Conditions

1. Position and direction of start and end points
2. Radii of allowed bends
3. Optimising for maximum radii
4. Length of the tube
5. Diameter of tubing



#### External Conditions

1. Not intersecting two pipes leaving at least 3-5mm (specified) between pipes

   1. Using pipes diameter to prevent this
2. Not intersecting fixed geometry within the space (why IIgnoreArea is framed out)
3. Simplifying all tubing to a potential minimum equal distance algorithm.





### Throwing Out a Base Idea

Got this base idea from a while ago that may help



#### 1\. The Geometry Model: Arc-Line-Arc

In C#, you’ll want to create a class structure that represents the pipe as a `List<Segment>`. A `Segment` is either a `Straight` (with a length) or an `Arc` (with a fixed radius and an angle).

* **Fixed CLR Constraint:** Your script should only allow the `Arc` segments to pull from your predefined list (e.g., 50mm, 75mm, 100mm).
* **The "Single-Planar" Bend:** In CNC terms, every individual bend is inherently "planar"—the tube sits in the die and wraps around it. The 3D complexity comes from the **Rotation** (R) between those bends.





#### 2\. The C# Logic: The "LRA" Solver



Optimizer will now "tune" three specific variables for each bend:

1. **Feed (L):** The length of the straight section before the bend.
2. **Rotation (R):** The degree the pipe is rotated in the chuck before the die closes.
3. **Angle (A):** How far the die actually wraps the pipe.





#### The Mathematical Objective

Your script needs to find the values of L, R, and A for each segment such that:

* The final coordinate matches the collector port.
* The final tangent vector matches the collector's entry angle.
* ∑L+∑(Arc Lengths)=Target Length.





#### 3\. Handling the "Fixed Radius" in Code



Since you have a fixed set of radii, your script can use a **"Point-and-Tangent"** method.

1. **Vector Entry/Exit:** Start with a vector from the exhaust port.
2. **The "Turn":** To change direction toward the collector, the script calculates a circle that is tangent to the current path. The radius of this circle **must** be one of your CLR values.
3. **The Intersection:** If you have two fixed points and two fixed vectors, there is a specific geometric solution called a **Bi-Arc**. In C#, you can solve for a Bi-Arc that connects two points using two arcs of your fixed CLR and one straight section in between.





#### 4\. The "Length Injection" Strategy

Since you can "cut and weld straight sections to any length," your script can be much simpler:

* **Step 1:** Solve for the most direct, "cleanest" path from port to collector using your fixed CLR arcs. This is your "Minimum Length Path."
* **Step 2:** Calculate the "Deficit" (Target Length - Minimum Length).
* **Step 3:** Look for a straight section in the path. "Break" that straight section and inject a **"U-bend"** or a **"C-loop"** (essentially two arcs and a straight).
* **Step 4:** Adjust the dimensions of that loop until the added length perfectly matches your deficit.







#### 5\. Implementation in C# (Logic Flow)



```
public class BendSegment {
    public double CLR { get; set; } // Fixed from your die set
    public double Angle { get; set; } // How much to bend
    public double Rotation { get; set; } // Rotation of the pipe in the machine
    public double StraightLength { get; set; } // Length of straight pipe before this bend
}

// Optimization Loop
public void OptimizeRunner(Vector3 start, Vector3 end, double targetLength) {
    // 1. Generate a basic path using a Bi-Arc algorithm
    var path = GenerateInitialPath(start, end, myFixedCLR);

    // 2. If path too short, find a straight section and 'evolve' it
    while (path.TotalLength < targetLength) {
        // Add a "bump" in the pipe to soak up length
        path.InjectLengthAdjustment(targetLength - path.TotalLength);
    }
    
    // 3. Export LRA Data
    ExportToBender(path.GetLRAData());
}

```







