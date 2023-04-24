namespace Middleware.OcelotGateway.Contracts
{
    public class LoginRequest
    {
        public Guid? Id { get; set; }
        public string? UserName { get; set; }
        public string Password { get; set; }
    }
}