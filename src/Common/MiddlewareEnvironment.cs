namespace Middleware.Common;

public class MiddlewareEnvironment : IEnvironment
{
    /// <inheritdoc />
    public string GetEnvVariable(string name)
    {
        return Environment.GetEnvironmentVariable(name);
    }
    /// <inheritdoc />
    public void SetEnvVariable(string name, string value)
    {
        Environment.SetEnvironmentVariable(name, value);
    }
    /// <inheritdoc />
    public bool DirectoryExists(string dir)
    {
        return Directory.Exists(dir);
    }
    /// <inheritdoc />
    public List<string> GetFileNamesInDir(string dir)
    {
        var list = new List<string>();
        if (DirectoryExists(dir) == false)
        {
            return list;
        }

        list.AddRange(Directory.GetFiles(dir).Select(Path.GetFileName));

        return list;
    }
    /// <inheritdoc />
    public bool FileExists(string file)
    {
        return File.Exists(file);
    }
}