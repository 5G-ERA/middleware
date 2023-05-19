namespace Middleware.ResourcePlanner.SliceManager;

public class SliceManager : ISliceManager
{
    private readonly ISliceManagerApi _sliceManagerApi;

    public SliceManager(ISliceManagerApi sliceManagerApi)
    {
        _sliceManagerApi = sliceManagerApi;
    }

    public async Task AttachImsiToSlice(string imsi, string sliceId, int dataRateUpLink, int dataRateDownLink)
    {
        var request = new AttachImsiToSliceRequest
        {
            Imsi = imsi,
            SliceId = sliceId,
            UsedDataRateDl = dataRateDownLink,
            UsedDataRateUl = dataRateUpLink
        };

        await _sliceManagerApi.AttachImsiToSlice(request);
    }
}