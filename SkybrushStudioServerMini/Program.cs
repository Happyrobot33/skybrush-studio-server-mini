using SkybrushStudioServerMini.Operations;
using SkybrushStudioServerMini.Queries;

var builder = WebApplication.CreateBuilder(args);
builder.Services.ConfigureHttpJsonOptions(options =>
{
    options.SerializerOptions.PropertyNamingPolicy = System.Text.Json.JsonNamingPolicy.CamelCase;
    options.SerializerOptions.WriteIndented = true;
    options.SerializerOptions.Converters.Add(new System.Text.Json.Serialization.JsonStringEnumConverter(System.Text.Json.JsonNamingPolicy.CamelCase));
});

builder.Services.AddRequestDecompression();

var app = builder.Build();

app.UseRequestDecompression();

MatchPoints matchPoints = new MatchPoints(app);
SkybrushStudioServerMini.Queries.Version versionQuery = new SkybrushStudioServerMini.Queries.Version(app);
SkybrushStudioServerMini.Queries.Limits limitsQuery = new SkybrushStudioServerMini.Queries.Limits(app);

app.MapGet("/ping", () => Results.Ok(new { result = true }));

app.MapFallback(async (HttpContext ctx) =>
{
    Console.WriteLine($"[unmatched] {ctx.Request.Method} {ctx.Request.Path}{ctx.Request.QueryString}");
    ctx.Response.StatusCode = 404;
    await ctx.Response.WriteAsync("Not Found");
});

app.Run();
