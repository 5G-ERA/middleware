namespace Middleware.TaskPlanner.Exceptions
{
    [Serializable]
    public class IncorrectROSDistroException : Exception
    {

        public IncorrectROSDistroException() : base($"The robot and the desired netApp to use do not have the same ROS Distro.")
        {

        }

    }
}
