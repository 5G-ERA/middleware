namespace Middleware.Models.Exceptions;

[Serializable]
public class IncorrectRosDistroNameException : Exception
{
    public IncorrectRosDistroNameException() : base("Cannot identify ROS distribution with a given name.")
    {
    }
}