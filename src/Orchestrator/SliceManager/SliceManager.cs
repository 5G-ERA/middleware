using Middleware.Orchestrator.SliceManager.Contracts;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.SliceManager;

internal class SliceManager : ISliceManager
{
    private readonly bool _isSliceManagerAvailable;
    private readonly ISliceManagerApi _sliceManagerApi;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

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

        //get the slice by the id
        SliceResponse sliceResponse = await _redisInterfaceClient.GetBySliceIdAsync(sliceId);

        if (sliceResponse == null) return; 
        var slice = sliceResponse.ToSlice();
        //attach the new imsi to the specific slice
        
        if (slice.Imsi != null) 
        {
            var exists = false;
            foreach (var item in slice.Imsi)
            {
                if (item == imsi) 
                {
                    exists = true;
                    break;
                }
            }    
            if (!exists)
            {
                slice.Imsi.Add(imsi);
            }
        }
        //store this slice in redis
        var sliceRequest = slice.ToSliceRequest();
        await _redisInterfaceClient.SliceAddAsync(sliceRequest);
    }
}