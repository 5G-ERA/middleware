namespace Middleware.Common;

public interface IEnvironment
{
    /// <summary>
    /// Gets the value from the environment variable
    /// </summary>
    /// <param name="name">name of the environment variable</param>
    /// <returns></returns>
    string GetEnvVariable(string name);
    /// <summary>
    /// Sets the value of the environment variable
    /// </summary>
    /// <param name="name">name of the variable</param>
    /// <param name="value">value of the variable</param>
    void SetEnvVariable(string name, string value);
    /// <summary>
    /// Check if the specified directory exists
    /// </summary>
    /// <param name="dir">full path to the directory</param>
    /// <returns></returns>
    bool DirectoryExists(string dir);
    /// <summary>
    /// Check if the specified file exists
    /// </summary>
    /// <param name="file">full path to the file</param>
    /// <returns></returns>
    bool FileExists(string file);
    /// <summary>
    /// Get only the names of the files in the specified directory
    /// </summary>
    /// <param name="dir">full path of the directory</param>
    /// <returns></returns>
    List<string> GetFileNamesInDir(string dir);
    
}