namespace Middleware.TaskPlanner.Exceptions
{
    [Serializable]
    public class IncorrectROSVersionException : Exception
    {
        public IncorrectROSVersionException() : base($"The robot and the desired netApp to use do not have the same ROS Version. ")
        {

        }

    }
}