using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Orchestrator.SliceManager.Contracts;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.SliceManager;

internal class SliceManager : ISliceManager
{
    private readonly bool _isSliceManagerAvailable;
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly ISliceManagerApi _sliceManagerApi;

    public SliceManager(ISliceManagerApi sliceManagerApi, bool isSliceManagerAvailable,
        IRedisInterfaceClient redisInterfaceClient)
    {
        _sliceManagerApi = sliceManagerApi;
        _isSliceManagerAvailable = isSliceManagerAvailable;
        _redisInterfaceClient = redisInterfaceClient;
    }

    public async Task AttachImsiToSlice(Guid robotId, string imsi, string sliceId, int dataRateUpLink,
        int dataRateDownLink)
    {
        if (!_isSliceManagerAvailable)
            return;

        var request = new AttachImsiToSliceRequest
        {
            Imsi = imsi,
            SliceId = sliceId,
            UsedDataRateDl = dataRateDownLink,
            UsedDataRateUl = dataRateUpLink
        };

        await _sliceManagerApi.AttachImsiToSlice(request);

        //get the slice by the id
        var slice = await GetSlice(sliceId);
        var robot = await GetValue(robotId);

        if (slice is null || robot is null) return;

        // check robot is connected to slice
        var connectedSlices =
            await _redisInterfaceClient.GetRelationAsync(robot, "CONNECTED_TO");


        if (connectedSlices is null || connectedSlices.Count == 0)
        {
            //attach the new imsi to the specific slice
            await ConnectRobotToSlice(robot, slice, imsi);
            return;
        }

        // when it is connected to the correct slice already
        if (connectedSlices.Count == 1 && connectedSlices.First().PointsTo.Name == sliceId)
            return;

        // when it is connected to a different slice[s]
        //      delete connection from the previous slice[s] (should always be connected to a single slice)
        foreach (var relation in connectedSlices)
        {
            await DeleteRobotConnectionToSlice(robot, imsi, relation.PointsTo.Name);
        }

        //      add connection to a new slice
        await ConnectRobotToSlice(robot, slice, imsi);
    }

    private async Task DeleteRobotConnectionToSlice(RobotModel robot, string imsi, string sliceName)
    {
        var sliceResp = await _redisInterfaceClient.GetBySliceIdAsync(sliceName);
        if (sliceResp is null) return;
        var slice = sliceResp.ToSlice();

        slice.Imsi.Remove(imsi);
        await UpdateSlice(slice);

        await _redisInterfaceClient.DeleteRelationAsync(robot, slice, "CONNECTED_TO");
    }

    private async Task ConnectRobotToSlice(RobotModel robot, SliceModel slice, string imsi)
    {
        slice.Imsi.Add(imsi);
        //store this slice in redis
        await UpdateSlice(slice);
        await _redisInterfaceClient.AddRelationAsync(robot, slice, "CONNECTED_TO");
    }

    private async Task UpdateSlice(SliceModel slice)
    {
        var sliceRequest = slice.ToSliceRequest();
        await _redisInterfaceClient.SliceAddAsync(sliceRequest);
    }

    private async Task<RobotModel> GetValue(Guid robotId)
    {
        var robotResp = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        var robot = robotResp?.ToRobot();
        return robot;
    }

    private async Task<SliceModel> GetSlice(string sliceId)
    {
        var sliceResponse = await _redisInterfaceClient.GetBySliceIdAsync(sliceId);
        var slice = sliceResponse?.ToSlice();
        return slice;
    }
}