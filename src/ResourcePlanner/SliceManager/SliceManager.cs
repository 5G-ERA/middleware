namespace Middleware.ResourcePlanner.SliceManager;

internal class SliceManager : ISliceManager
{
    private readonly bool _isSliceManagerAvailable;
    private readonly ISliceManagerApi _sliceManagerApi;

    public SliceManager(ISliceManagerApi sliceManagerApi, bool isSliceManagerAvailable)
    {
        _sliceManagerApi = sliceManagerApi;
        _isSliceManagerAvailable = isSliceManagerAvailable;
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