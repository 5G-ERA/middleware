namespace Middleware.TaskPlanner.Exceptions;

[Serializable]
public class IncorrectLocationException : Exception
{
    public string LocationName { get; }
    public string LocationType { get; }
    
    public IncorrectLocationException(string locationName, string locationType) 
        : base($"Could not obtain location with name '{locationName}' and type '{locationType}' within organization.")
    {
        LocationType = locationType;
        LocationName = locationName;
    }

}