using System.Reflection;

namespace SkybrushStudioServerMini.Queries
{
    class Version
    {
        public Version(WebApplication app)
        {
            app.MapGet("/queries/version", () =>
            {
                var version = Assembly.GetExecutingAssembly()
                    .GetCustomAttribute<AssemblyInformationalVersionAttribute>()
                    ?.InformationalVersion ?? "0.0.0";

                return Results.Ok(new { version });
            });
        }
    }
}
