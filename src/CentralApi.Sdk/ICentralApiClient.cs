﻿using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;

namespace Middleware.CentralApi.Sdk;

public interface ICentralApiClient
{
    /// <summary>
    ///     Gets all available (online) Middleware locations
    /// </summary>
    /// <returns></returns>
    Task<LocationsResponse?> GetAvailableLocations();

    /// <summary>
    ///     Gets all available (online) Middleware locations for the specified Organization
    /// </summary>
    /// <returns></returns>
    Task<LocationsResponse?> GetAvailableLocations(string orgName);

    /// <summary>
    ///     Sets specified location available (online) to be communicated to
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    Task<LocationResponse?> RegisterLocation(RegisterRequest request);

    /// <summary>
    ///     Sets specified location unavailable (offline)
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    Task DeRegisterLocation(RegisterRequest request);

    Task SetStatus(CloudEdgeStatusRequest request, Guid id);
}