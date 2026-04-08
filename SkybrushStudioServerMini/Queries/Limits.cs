namespace SkybrushStudioServerMini.Queries
{
    class Limits
    {
        public Limits(WebApplication app)
        {
            app.MapGet("/queries/limits", () =>
            {
                return Results.Ok(new
                {
                    num_drones = 200,
                    features = new[] { "export:plot" },
                    timeout = 30
                });
            });
        }
    }
}
