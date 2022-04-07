namespace Middleware.Orchestrator.Exceptions;

[Serializable]
public class UnableToParseYamlConfigException : IncorrectDataException
{
    /// <summary>
    /// Name of the image with invalid configuration
    /// </summary>
    public string ImageName { get; set; }
    /// <summary>
    /// Name of the invalid property
    /// </summary>
    public string PropertyName { get; set; }
    public UnableToParseYamlConfigException(string imageName, string propertyName) : base($"Unable to parse the yaml configuration for {propertyName} at {imageName}")
    {
        ImageName = imageName;
        PropertyName = propertyName;
    }
}