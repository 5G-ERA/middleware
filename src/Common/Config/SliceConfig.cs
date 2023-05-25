namespace Middleware.Common.Config;

public class SliceConfig
{
    /// <summary>
    ///     Name of the section in the appsettings.json file
    /// </summary>
    public const string ConfigName = "Slice";

    public string Hostname { get; init; }
}