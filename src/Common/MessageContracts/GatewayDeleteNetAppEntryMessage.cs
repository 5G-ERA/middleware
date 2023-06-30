using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Middleware.Common.MessageContracts;
public record GatewayDeleteNetAppEntryMessage : Message
{
    public Guid ActionPlanId { get; set; }

    public Guid ServiceInstanceId { get; set; }

    public string NetAppName { get; set; }

    public string DeploymentLocation { get; init; }
}
