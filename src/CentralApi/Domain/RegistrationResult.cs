namespace Middleware.CentralApi.Domain;

public class RegistrationResult
{
    public bool Succeeded { get; }

    public RegistrationResult(bool succeeded )
    {
        Succeeded = succeeded;
    }
}