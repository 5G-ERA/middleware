namespace Middleware.Common.Helpers
{
    public static class ListHelpers
    {
        public static bool IsNullOrEmpty<T>(List<T> list)
        {
            return list == null || list.Count == 0;
        }
    }
}
