namespace Middleware.OcelotGateway.Contracts
{
    public class RegisterResponse
    {
        public Guid Id { get; set; }
        public string UserName { get; set; }
        public string Role { get; set; }
    }
}