namespace Middleware.Models.Domain;

public struct RosDistro
{
    public static RosDistro Electric { get; } = new("Electric", 1);
    public static RosDistro Groovy { get; } = new("Groovy", 1);
    public static RosDistro Fuerte { get; } = new("Fuerte", 1);
    public static RosDistro Hydro { get; } = new("Hydro", 1);
    public static RosDistro Indigo { get; } = new("Indigo", 1);
    public static RosDistro Kinetic { get; } = new("Kinetic", 1);
    public static RosDistro Lunar { get; } = new("Lunar", 1);
    public static RosDistro Noetic { get; } = new("Noetic", 1);
    public static RosDistro Ardent { get; } = new("Ardent", 2);
    public static RosDistro Dashing { get; } = new("Dashing", 2);
    public static RosDistro Bouncy { get; } = new("Bouncy", 2);
    public static RosDistro Crystal { get; } = new("Crystal", 2);
    public static RosDistro Humble { get; } = new("Humble", 2);
    public static RosDistro Eloquent { get; } = new("Eloquent", 2);
    public static RosDistro Melodic { get; } = new("Melodic", 2);
    public static RosDistro Galactic { get; } = new("Galactic", 2);
    public static RosDistro Foxy { get; } = new("Foxy", 2);

    public RosDistro(string name, short rosVersion)
    {
        Name = name;
        RosVersion = rosVersion;
    }

    public string Name { get; }
    public short RosVersion { get; set; }
}

public static class RosDistroHelper
{
    private static readonly List<RosDistro> Distros = new()
    {
        RosDistro.Electric,
        RosDistro.Groovy,
        RosDistro.Fuerte,
        RosDistro.Hydro,
        RosDistro.Indigo,
        RosDistro.Kinetic,
        RosDistro.Lunar,
        RosDistro.Noetic,
        RosDistro.Ardent,
        RosDistro.Dashing,
        RosDistro.Crystal,
        RosDistro.Humble,
        RosDistro.Foxy,
        RosDistro.Melodic,
        RosDistro.Bouncy,
        RosDistro.Eloquent,
        RosDistro.Galactic
    };


    public static RosDistro? FromName(string name)
    {
        var sanName = name.ToLower();
        var distro = Distros.FirstOrDefault(d => d.Name.ToLower() == sanName);
        return distro;
    }
}