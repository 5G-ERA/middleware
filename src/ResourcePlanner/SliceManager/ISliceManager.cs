namespace Middleware.ResourcePlanner.SliceManager
{
    public interface ISliceManager
    {
        Task RegisterUrllcSlice(string slice);

        Task RegisterEmbbSlice(string slice);
    }
}
