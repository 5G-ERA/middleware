using System.Security.Cryptography;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;

namespace Middleware.CentralApi.Auth;

public class ApiKeyService : IApiKeyService
{
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;
    private readonly IRobotRepository _robotRepository;

    public ApiKeyService(IEdgeRepository edgeRepository, ICloudRepository cloudRepository,
        IRobotRepository robotRepository)
    {
        _edgeRepository = edgeRepository;
        _cloudRepository = cloudRepository;
        _robotRepository = robotRepository;
    }

    public string GenerateApiKey(ApiKeyUserType userType)
    {
        var prefix = userType switch
        {
            ApiKeyUserType.Location => "MW-L-",
            ApiKeyUserType.Robot => "MW-R-",
            _ => throw new ArgumentOutOfRangeException(nameof(userType), userType, null)
        };

        var bytes = RandomNumberGenerator.GetBytes(32);

        var base64String = Convert.ToBase64String(bytes)
            .Replace("/", "")
            .Replace("+", "");


        var keyLength = 32 - prefix.Length;

        return prefix + base64String[..keyLength];
    }

    public bool ValidateApiKeyFormat(string apiKey)
    {
        if (string.IsNullOrWhiteSpace(apiKey)) return false;

        if (!apiKey.StartsWith("MW-")) return false;

        var userType = GetApiKeyUserType(apiKey);

        if (userType is null) return false;

        return true;
    }

    /// <inheritdoc />
    public async Task<Dictionary<string, Guid>> GetClientKeys()
    {
        var keys = new Dictionary<string, Guid>();

        var edges = await _edgeRepository.GetAllAsync();

        foreach (var edge in edges)
        {
            if (string.IsNullOrWhiteSpace(edge.ApiKey))
            {
                edge.ApiKey = GenerateApiKey(ApiKeyUserType.Location);
                await _edgeRepository.UpdateAsync(edge);
            }

            keys.Add(edge.ApiKey, edge.Id);
        }

        var clouds = await _cloudRepository.GetAllAsync();

        foreach (var cloud in clouds)
        {
            if (string.IsNullOrWhiteSpace(cloud.ApiKey))
            {
                cloud.ApiKey = GenerateApiKey(ApiKeyUserType.Location);
                await _cloudRepository.UpdateAsync(cloud);
            }

            keys.Add(cloud.ApiKey, cloud.Id);
        }

        var robots = await _robotRepository.GetAllAsync();

        foreach (var robot in robots)
        {
            if (string.IsNullOrWhiteSpace(robot.ApiKey))
            {
                robot.ApiKey = GenerateApiKey(ApiKeyUserType.Robot);
                await _robotRepository.UpdateAsync(robot);
            }

            keys.Add(robot.ApiKey, robot.Id);
        }

        return keys;
    }

    private ApiKeyUserType? GetApiKeyUserType(string apiKey)
    {
        return apiKey switch
        {
            _ when apiKey.StartsWith("MW-L-") => ApiKeyUserType.Location,
            _ when apiKey.StartsWith("MW-R-") => ApiKeyUserType.Robot,
            _ => null
        };
    }
}