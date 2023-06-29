using Middleware.Models.Exceptions;

namespace Middleware.Models.Domain;

public enum RosVersion
{
    Ros1 = 1,
    Ros2 = 2
}

public struct RosDistro
{
    public static RosDistro Electric { get; } = new("Electric", RosVersion.Ros1);
    public static RosDistro Groovy { get; } = new("Groovy", RosVersion.Ros1);
    public static RosDistro Fuerte { get; } = new("Fuerte", RosVersion.Ros1);
    public static RosDistro Hydro { get; } = new("Hydro", RosVersion.Ros1);
    public static RosDistro Indigo { get; } = new("Indigo", RosVersion.Ros1);
    public static RosDistro Kinetic { get; } = new("Kinetic", RosVersion.Ros1);
    public static RosDistro Lunar { get; } = new("Lunar", RosVersion.Ros1);
    public static RosDistro Noetic { get; } = new("Noetic", RosVersion.Ros1);
    public static RosDistro Ardent { get; } = new("Ardent", RosVersion.Ros2);
    public static RosDistro Dashing { get; } = new("Dashing", RosVersion.Ros2);
    public static RosDistro Bouncy { get; } = new("Bouncy", RosVersion.Ros2);
    public static RosDistro Crystal { get; } = new("Crystal", RosVersion.Ros2);
    public static RosDistro Humble { get; } = new("Humble", RosVersion.Ros2);
    public static RosDistro Eloquent { get; } = new("Eloquent", RosVersion.Ros2);
    public static RosDistro Melodic { get; } = new("Melodic", RosVersion.Ros2);
    public static RosDistro Galactic { get; } = new("Galactic", RosVersion.Ros2);
    public static RosDistro Foxy { get; } = new("Foxy", RosVersion.Ros2);

    public RosDistro(string name, short rosVersion)
    {
        Name = name;
        RosVersion = (RosVersion)rosVersion;
        RosVersionInt = rosVersion;
    }

    public RosDistro(string name, RosVersion rosVersion)
    {
        Name = name;
        RosVersion = rosVersion;
        RosVersionInt = (short)rosVersion;
    }

    public string Name { get; }
    public short RosVersionInt { get; set; }
    public RosVersion RosVersion { get; set; }
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

    /// <summary>
    ///     Parses the name of the ROS distribution to return information about ros version
    /// </summary>
    /// <param name="name"></param>
    /// <exception cref="IncorrectRosDistroNameException"></exception>
    /// <returns></returns>
    public static RosDistro FromName(string name)
    {
        if (name == null) throw new ArgumentNullException(nameof(name));
        var sanName = name.ToLower();
        var distro = Distros.FirstOrDefault(d => d.Name.ToLower() == sanName);
        return distro;
    }

    /// <summary>
    ///     Return all possible ROS distribution names
    /// </summary>
    /// <returns></returns>
    public static IReadOnlyList<string> GetRosDistroNames()
    {
        return Distros.Select(d => d.Name).ToList();
    }

    /// <summary>
    ///     Return ROS distribution names for specified ROS version
    /// </summary>
    /// <returns></returns>
    public static IReadOnlyList<string> GetRosDistroNamesByVersion(RosVersion rosVersion)
    {
        return Distros.Where(d => d.RosVersion == rosVersion).Select(d => d.Name).ToList();
    }
}