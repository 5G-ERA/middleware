using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class ActionModel
{
  [JsonPropertyName("ActionId")]
  public Guid ActionId { get; set; }

  [JsonPropertyName("Order")]
  public int Order { get; set; }

  [JsonPropertyName("Placement")]
  public string Placement { get; set; }

  [JsonPropertyName("ActionPriority")]
  public string ActionPriority { get; set; }

  [JsonPropertyName("Services")]
  public List<ServiceDataModel> Services { get; set; }
}