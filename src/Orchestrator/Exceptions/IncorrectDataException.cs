namespace Middleware.Orchestrator.Exceptions;

[Serializable]
public class IncorrectDataException : Exception
{
    public string Reason { get; private set; }
    public IncorrectDataException(string reason) : base($"The data configuration for service is incorrect. {reason}")
    {
        Reason = reason;
    }
}
