﻿using k8s.Models;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros2ConnectionBuilder : IRosConnectionBuilder
{
    private const short Ros2 = 2;
    private readonly RosDistro _distro;

    public Ros2ConnectionBuilder(RosDistro distro)
    {
        if ((int)distro != Ros2)
        {
            throw new ArgumentException(
                "Ros1ConnectionBuilder cannot provide connectivity for ROs version other than 1", nameof(distro));
        }

        _distro = distro;
        RosVersion = (int)distro;
        RosDistro = distro.ToString();
    }


    /// <inheritdoc />
    public int RosVersion { get; }

    /// <inheritdoc />
    public string RosDistro { get; }

    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl)
    {
        //TODO: we have to figure out how to enable ros2. It is possible that it will be same as ros1, but we will see :)
        return dpl;
    }
}