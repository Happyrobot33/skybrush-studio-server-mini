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
                // Response mapping is target-indexed: mapping[targetIndex] = sourceIndex
                var rawMapping = new int?[request.target.Length];
                switch (request.method)
                {
                    case MatchingMethod.Fixed:
                        for (int i = 0; i < rawMapping.Length; i++)
                        {
                            rawMapping[i] = i < request.source.Length ? i : (int?)null;
                        }
                        break;
                    case MatchingMethod.Optimal:
                        rawMapping = OptimalMapping(request.source, request.target);
                        break;
                }

                var radii = ParseRadii(request.radius, request.source.Length);
                float? clearance = radii is null
                    ? null
                    : ComputeClearance(rawMapping, request.source, request.target, radii);

                var mapping = NormalizeMapping(rawMapping, request.source, request.target);
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

            int?[] mapping = new int?[m];
            for (int j = 0; j < m; j++)
                mapping[j] = null;

            for (int i = 0; i < n; i++)
            {
                int j = rowAssign[i];
                if (j >= 0 && j < m)
                    mapping[j] = i;
            }

            return mapping;
        }

        private static int?[] NormalizeMapping(int?[] mapping, Point[] source, Point[] target)
        {
            var normalized = new int?[target.Length];
            for (int targetIndex = 0; targetIndex < target.Length; targetIndex++)
            {
                if (targetIndex >= mapping.Length || !mapping[targetIndex].HasValue)
                {
                    normalized[targetIndex] = null;
                    continue;
                }

                int sourceIndex = mapping[targetIndex]!.Value;
                if (sourceIndex < 0 || sourceIndex >= source.Length)
                {
                    normalized[targetIndex] = null;
                    continue;
                }

                normalized[targetIndex] = sourceIndex;
            }

            return normalized;
        }

        private static float[]? ParseRadii(JsonElement? radiusElement, int droneCount)
        {
            if (!radiusElement.HasValue)
                return null;

            JsonElement radius = radiusElement.Value;
            if (radius.ValueKind == JsonValueKind.Null || radius.ValueKind == JsonValueKind.Undefined)
                return null;

            if (radius.ValueKind == JsonValueKind.Number)
            {
                float value = radius.GetSingle();
                var radii = new float[droneCount];
                for (int i = 0; i < droneCount; i++)
                    radii[i] = value;
                return radii;
            }

            if (radius.ValueKind == JsonValueKind.Array)
            {
                var values = radius.EnumerateArray()
                    .Where(e => e.ValueKind == JsonValueKind.Number)
                    .Select(e => e.GetSingle())
                    .ToArray();

                if (values.Length == 0)
                    return null;

                var radii = new float[droneCount];
                for (int i = 0; i < droneCount; i++)
                    radii[i] = i < values.Length ? values[i] : values[^1];
                return radii;
            }

            return null;
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
            for (int i = 0; i < n; i++)
                rowAssign[i] = -1;
            for (int j = 1; j <= n; j++)
                if (p[j] != 0)
                    rowAssign[p[j] - 1] = j - 1;
            return rowAssign;
        }

        private static float ComputeClearance(int?[] mapping, Point[] source, Point[] target, float[] radii)
        {
            // Collect only matched (source, target) pairs from target-indexed mapping.
            var pairs = mapping
                .Select((srcIdx, tgtIdx) => (srcIdx, tgtIdx))
                .Where(p => p.srcIdx.HasValue)
                .Select(p => (srcIdx: p.srcIdx!.Value, tgtIdx: p.tgtIdx))
                .Where(p => p.srcIdx >= 0 && p.srcIdx < source.Length && p.tgtIdx >= 0 && p.tgtIdx < target.Length)
                .ToList();

            if (pairs.Count < 2)
                return 0f;

            // Minimum separation between straight-line constant-velocity trajectories
            double minDist = double.MaxValue;
            for (int i = 0; i < pairs.Count; i++)
            {
                for (int j = i + 1; j < pairs.Count; j++)
                {
                    var a = pairs[i];
                    var b = pairs[j];
                    double separation = MinTrajectorySeparation(
                        source[a.srcIdx],
                        target[a.tgtIdx],
                        source[b.srcIdx],
                        target[b.tgtIdx]);

                    double radiusA = a.srcIdx < radii.Length ? radii[a.srcIdx] : 0.0;
                    double radiusB = b.srcIdx < radii.Length ? radii[b.srcIdx] : 0.0;
                    minDist = Math.Min(minDist, separation - radiusA - radiusB);
                }
            }

            return (float)minDist;
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

        private static bool PointsEqual(Point a, Point b, double epsilon = 1e-6)
        {
            if (a.coords.Length != b.coords.Length)
                return false;

            for (int d = 0; d < a.coords.Length; d++)
            {
                if (Math.Abs(a.coords[d] - b.coords[d]) > epsilon)
                    return false;
            }

            return true;
        }

        record Request
        {
            public required Point[] source { get; set; }
            public required Point[] target { get; set; }
            public JsonElement? radius { get; set; }
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
            public float? clearance { get; set; }
        }

        enum MatchingMethod
        {
            Optimal,
            Fixed,
        }
    }
}
