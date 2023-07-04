namespace Middleware.Common.ExtensionMethods;

public static class ListExtensions
{
    public static List<T> CreateCopy<T>(this List<T> source)
    {
        var count = source.Count;
        var newArray = new T[count];
        source.CopyTo(newArray);
        return newArray.ToList();
    }
}