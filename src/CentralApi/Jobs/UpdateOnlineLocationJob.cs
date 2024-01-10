using Middleware.Common.Job;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Quartz;

namespace Middleware.CentralApi.Jobs;

[DisallowConcurrentExecution]
public class UpdateOnlineLocationJob : BaseJob<UpdateOnlineStatusJob>
{
    private readonly ILogger _logger;
    private readonly IRobotRepository _robotRepository;
    private readonly ISystemConfigRepository _systemConfig;

    public UpdateOnlineLocationJob(ILogger<UpdateOnlineStatusJob> logger, IRobotRepository robotRepository,
        ISystemConfigRepository systemConfig) : base(logger)
    {
        _logger = logger;
        _robotRepository = robotRepository;
        _systemConfig = systemConfig;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {
        var cfg = await _systemConfig.GetConfigAsync();
        if (cfg is null) throw new ArgumentException("No system config was found");

        var heartbeatExpiration = cfg.HeartbeatExpirationInMinutes;
        // get all CAN_REACH relations
        var relations = new List<RelationModel>();
        try
        {
            relations = await _robotRepository.GetRelationsWithName("CAN_REACH");
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "There was en error while getting relations: CAN_REACH;");
        }

        if (relations.Any())
            // check all relations
        {
            foreach (var rel in relations)
            {
                var relationAttributes = rel.RelationAttributes;
                var robotId = rel.InitiatesFrom.Id;
                var locationId = rel.PointsTo.Id;
                if (relationAttributes != null)
                {
                    foreach (var relAttribute in relationAttributes)
                    {
                        if (relAttribute.Key == "lastUpdatedTime")
                        {
                            var dateInput = (string)relAttribute.Value;
                            var parsedDateTime = DateTime.Parse(dateInput);

                            var threeMinutesEarlier = DateTime.UtcNow.AddMinutes(-1 * heartbeatExpiration);
                            // Check if last updated time was no later than 3 minutes ago from now.
                            if (parsedDateTime < threeMinutesEarlier)
                            {
                                var relationModel = new RelationModel();
                                var relationName = rel.RelationName;
                                relationModel.InitiatesFrom.Id = robotId;
                                relationModel.InitiatesFrom.Type = rel.InitiatesFrom.Type;
                                relationModel.RelationName = relationName;
                                relationModel.PointsTo.Id = locationId;
                                relationModel.PointsTo.Type = rel.PointsTo.Type;
                                try
                                {
                                    var isValid = await _robotRepository.DeleteRelationAsync(relationModel);
                                    if (!isValid)
                                    {
                                        Logger.LogError(
                                            "Deleting relation did not succeed: formId: {robotId} relationName: {name} toId:{Id2}",
                                            robotId, relationName, locationId);
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Logger.LogError(ex,
                                        "There was en error while deleting relation formId: {robotId} relationName: {name} toId:{Id2}",
                                        robotId, relationName, locationId);
                                    //throw new ArgumentException("The relation was not deleted", nameof(model));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}