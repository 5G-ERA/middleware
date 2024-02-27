using System.Net;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Options;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.OcelotGateway.Contracts;
using Middleware.OcelotGateway.Services;

namespace Middleware.OcelotGateway.Controllers;

[Route("api/v1")]
[ApiController]
public class LoginController : MiddlewareController
{
    private readonly IOptions<JwtConfig> _jwtConfig;
    private readonly ILogger _logger;
    private readonly IUserRepository _userRepository;


    public LoginController(IUserRepository userRepository, IOptions<JwtConfig> jwtConfig,
        ILogger<LoginController> logger)
    {
        _userRepository = userRepository ?? throw new ArgumentNullException(nameof(userRepository));
        _jwtConfig = jwtConfig ?? throw new ArgumentNullException(nameof(jwtConfig));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }


    /// <summary>
    ///     Register a new user into the system
    /// </summary>
    /// <param name="request"></param>
    /// <returns> HttpStatusCode Created </returns>
    [AllowAnonymous]
    [HttpPost]
    [Route("register", Name = "Register")]
    [ProducesResponseType(typeof(RegisterResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> Register([FromBody] RegisterRequest request)
    {
        if (request == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Please enter valid credentials"));
        try
        {
            var user = request.ToUser();
            var salt = AuthHelper.GetSalt();
            user.Salt = AuthHelper.StringifySalt(salt);

            user.Password = AuthHelper.HashPasswordWithSalt(user.Password, salt);

            await _userRepository.AddAsync(user, () => user.Id);

            return Ok(user.ToRegisterResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Login the user into the system
    /// </summary>
    /// <param name="login"></param>
    /// <returns> JWT Token with expiry date </returns>
    [AllowAnonymous]
    [HttpPost]
    [Route("login", Name = "Login")]
    [ProducesResponseType(typeof(TokenModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.Unauthorized)]
    public async Task<IActionResult> Login([FromBody] LoginRequest login)
    {
        if (login == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Credentials must be provided"));
        try
        {
            IActionResult response = Unauthorized();

            var (authenticated, user) = await AuthenticateUser(login);
            if (authenticated)
            {
                var token = new TokenService(_jwtConfig.Value);
                var newToken = token.GenerateToken(user.Id, user.Role);
                response = Ok(newToken);
            }

            return response;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, $"Unauthorized, username or password are incorrect:");
            return ErrorMessageResponse(HttpStatusCode.Unauthorized, "unauthorized",
                $"Unauthorized, username or password are incorrect: {ex.Message}");
        }
    }


    /// <summary>
    ///     Checks for user credentials
    /// </summary>
    /// <param name="login"></param>
    /// <returns> True or False according to users' credentials </returns>
    private async Task<Tuple<bool, UserModel>> AuthenticateUser(LoginRequest login)
    {
        try
        {
            var user = login.UserName is null
                ? await _userRepository.GetByIdAsync(login.Id!.Value)
                : await _userRepository.FindSingleAsync(u => u.UserName == login.UserName);

            if (user == null) return new(false, null);

            var salt = Convert.FromBase64String(user.Salt);

            var computedHashedPassword = AuthHelper.HashPasswordWithSalt(login.Password, salt);

            return computedHashedPassword.Equals(user.Password) ? new(true, user) : new(false, null);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error has occured during the authentication process");
            return new(false, null);
        }
    }
}