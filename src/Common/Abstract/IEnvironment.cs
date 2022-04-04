namespace Middleware.Common;

public interface IEnvironment
{
    string GetEnvVariable(string name);
    void SetEnvVariable(string name, string value);
}