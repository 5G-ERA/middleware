using Refit;

namespace Middleware.ResourcePlanner.SliceManager;

public interface ISliceManagerApi
{
    [Post("/api/v1/sliceInventory/SB/reportSliceParameters/urllc")]
    Task RegisterUrllcSlice(string slice);

    [Post("/api/v1/sliceInventory/SB/reportSliceParameters/embb")]
    Task RegisterEmbbSlice(string slice);

    [Post("/testbed/sliceUE/attach")]
    Task AttachImsiToSlice([Body] AttachImsiToSliceRequest request);
}