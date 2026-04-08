using System.Text.Json;
using System.Text.Json.Serialization;

namespace SkybrushStudioServerMini.Operations
{
    class MatchPoints
    {
        public MatchPoints(WebApplication app)
        {
            app.MapPost("/operations/match-points", (Request request) =>
            {
                //if fixed mapping, just 1:1 map the source to target
                var mapping = new int?[request.source.Length];
                switch (request.method)
                {
                    case MatchingMethod.Fixed:
                        for (int i = 0; i < mapping.Length; i++)
                        {
                            mapping[i] = i < request.target.Length ? i : (int?)null;
                        }
                        break;
                    case MatchingMethod.Optimal:
                        mapping = OptimalMapping(request.source, request.target);
                        break;
                }
                float clearance = ComputeClearance(mapping, request.source, request.target, request.radius);
                var result = new Response
                {
                    version = request.version,
                    mapping = mapping,
                    clearance = clearance
                };
                return Results.Ok(result);
            });
        }

        private static int?[] OptimalMapping(Point[] source, Point[] target)
        {
            int n = source.Length;
            int m = target.Length;
            int dim = Math.Max(n, m);

            double[,] cost = new double[dim, dim];
            const double Inf = 1e18;
            for (int i = 0; i < dim; i++)
                for (int j = 0; j < dim; j++)
                    cost[i, j] = (i < n && j < m) ? EuclideanDistance(source[i], target[j]) : Inf;

            int[] rowAssign = RunHungarian(cost, dim);

            int?[] mapping = new int?[n];
            for (int i = 0; i < n; i++)
            {
                int j = rowAssign[i];
                if (j < m)
                    mapping[i] = j;
            }
            return mapping;
        }

        // Jonker-Volgenant style Hungarian algorithm, O(n^3)
        private static int[] RunHungarian(double[,] a, int n)
        {
            double[] u = new double[n + 1];
            double[] v = new double[n + 1];
            int[] p = new int[n + 1]; // p[j] = row assigned to column j (1-indexed)
            int[] way = new int[n + 1];

            for (int i = 1; i <= n; i++)
            {
                p[0] = i;
                int j0 = 0;
                double[] minVal = new double[n + 1];
                bool[] used = new bool[n + 1];
                for (int j = 0; j <= n; j++) minVal[j] = double.MaxValue;

                do
                {
                    used[j0] = true;
                    int i0 = p[j0];
                    double delta = double.MaxValue;
                    int j1 = -1;

                    for (int j = 1; j <= n; j++)
                    {
                        if (!used[j])
                        {
                            double val = a[i0 - 1, j - 1] - u[i0] - v[j];
                            if (val < minVal[j]) { minVal[j] = val; way[j] = j0; }
                            if (minVal[j] < delta) { delta = minVal[j]; j1 = j; }
                        }
                    }

                    for (int j = 0; j <= n; j++)
                    {
                        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                        else minVal[j] -= delta;
                    }

                    j0 = j1;
                } while (p[j0] != 0);

                do
                {
                    int j1 = way[j0];
                    p[j0] = p[j1];
                    j0 = j1;
                } while (j0 != 0);
            }

            int[] rowAssign = new int[n];
            for (int j = 1; j <= n; j++)
                if (p[j] != 0)
                    rowAssign[p[j] - 1] = j - 1;
            return rowAssign;
        }

        private static float ComputeClearance(int?[] mapping, Point[] source, Point[] target, float? radius)
        {
            // Collect only matched (source, target) pairs
            var pairs = mapping
                .Select((t, i) => (srcIdx: i, tgtIdx: t))
                .Where(p => p.tgtIdx.HasValue)
                .Select(p => (src: source[p.srcIdx], tgt: target[p.tgtIdx!.Value]))
                .ToList();

            if (pairs.Count < 2)
                return 0f;

            // Minimum separation between straight-line constant-velocity trajectories
            double minDist = double.MaxValue;
            for (int i = 0; i < pairs.Count; i++)
            {
                for (int j = i + 1; j < pairs.Count; j++)
                {
                    minDist = Math.Min(minDist, MinTrajectorySeparation(pairs[i].src, pairs[i].tgt, pairs[j].src, pairs[j].tgt));
                }
            }

            double r = radius ?? 0.0;
            return (float)(minDist - 2.0 * r);
        }

        // Minimum Euclidean distance between two linear trajectories over t in [0,1]
        private static double MinTrajectorySeparation(Point srcA, Point tgtA, Point srcB, Point tgtB)
        {
            int dims = srcA.coords.Length;
            // delta(t) = A + t*D  where A = srcA - srcB, D = (tgtA - tgtB) - (srcA - srcB)
            double[] A = new double[dims];
            double[] D = new double[dims];
            for (int d = 0; d < dims; d++)
            {
                A[d] = srcA.coords[d] - srcB.coords[d];
                D[d] = (tgtA.coords[d] - tgtB.coords[d]) - A[d];
            }

            double DD = 0, AD = 0, AA = 0;
            for (int d = 0; d < dims; d++)
            {
                DD += D[d] * D[d];
                AD += A[d] * D[d];
                AA += A[d] * A[d];
            }

            double t = DD > 0 ? Math.Clamp(-AD / DD, 0.0, 1.0) : 0.0;

            double distSq = 0;
            for (int d = 0; d < dims; d++)
            {
                double v = A[d] + t * D[d];
                distSq += v * v;
            }
            return Math.Sqrt(distSq);
        }

        private static double EuclideanDistance(Point a, Point b)
        {
            double sum = 0;
            int dims = Math.Min(a.coords.Length, b.coords.Length);
            for (int d = 0; d < dims; d++)
            {
                double diff = a.coords[d] - b.coords[d];
                sum += diff * diff;
            }
            return Math.Sqrt(sum);
        }

        record Request
        {
            public required Point[] source { get; set; }
            public required Point[] target { get; set; }
            public float? radius { get; set; }
            public MatchingMethod method { get; set; }
            public int version { get; set; }
        }

        [JsonConverter(typeof(PointConverter))]
        record Point(float[] coords);

        class PointConverter : JsonConverter<Point>
        {
            public override Point Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
            {
                var coords = JsonSerializer.Deserialize<float[]>(ref reader, options) ?? [];
                return new Point(coords);
            }

            public override void Write(Utf8JsonWriter writer, Point value, JsonSerializerOptions options)
            {
                JsonSerializer.Serialize(writer, value.coords, options);
            }
        }


        record Response
        {
            public int version { get; set; }
            public required int?[] mapping { get; set; }
            public float clearance { get; set; }
        }

        enum MatchingMethod
        {
            Optimal,
            Fixed,
        }
    }
}
