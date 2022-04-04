using System;
namespace Middleware.Common;

public class MiddlewareEnvironment : IEnvironment
{
    public string GetEnvVariable(string name)
    {
        return Environment.GetEnvironmentVariable(name);
    }

    public void SetEnvVariable(string name, string value)
    {
        Environment.SetEnvironmentVariable(name, value);
    }
}