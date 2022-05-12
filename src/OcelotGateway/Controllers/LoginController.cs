using System.Net;
using System.Security.Cryptography;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Cryptography.KeyDerivation;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using Middleware.OcelotGateway.Services;

namespace Middleware.OcelotGateway.Controllers
{
    [Route("api/v1")]
    [ApiController]
    public class LoginController : ControllerBase
    {
        private readonly IUserRepository _userRepository;
        private readonly ILogger _logger;
        

        public LoginController(IUserRepository userRepository, ILogger<LoginController> logger)
        {
            _userRepository = userRepository ?? throw new ArgumentNullException(nameof(userRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));   
        }


        /// <summary>
        /// Register a new user into the system
        /// </summary>
        /// <param name="register"></param>
        /// <returns> HttpStatusCode Created </returns>
        [AllowAnonymous]
        [HttpPost]
        [Route("register", Name = "Register")]
        [ProducesResponseType(typeof(UserModel), (int)HttpStatusCode.Created)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<UserModel>> Register([FromBody] UserModel register)
        {
            if (register == null)
            {
                return BadRequest("Please enter valid credentials");
            }
            try
            {
                byte[] salt = RandomNumberGenerator.GetBytes(128 / 8);
                register.Salt = Convert.ToBase64String(salt);

                register.Password = ComputeHashPassword(register.Password, salt);
        
                await _userRepository.AddAsync(register, () => register.Id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return StatusCode((int)HttpStatusCode.Created);
        }

        /// <summary>
        /// Login the user into the system
        /// </summary>
        /// <param name="login"></param>
        /// <returns> JWT Token with expiry date </returns>
        [AllowAnonymous]
        [HttpPost]
        [Route("login", Name = "Login")]
        [ProducesResponseType(typeof(TokenService), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.Unauthorized)]
        public async Task<IActionResult> Login([FromBody] UserModel login) 
        {
            if (login == null)
            {
                return BadRequest("Credentials must be provided");
            }
            try
            {
                IActionResult response = Unauthorized();

                bool authenticated = await AuthenticateUser(login);
                if (authenticated)
                {
                    TokenService token = new TokenService();
                    var newToken = token.GenerateToken(login.Id);
                    response = Ok(newToken);
                }
                return response;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Unauthorized("User Unauthorized, username or password are incorect");
            }
            
        }


        /// <summary>
        /// Checks for user credentials
        /// </summary>
        /// <param name="login"></param>
        /// <returns> True or False according to users' credentials </returns>
        private async Task<bool> AuthenticateUser(UserModel login) 
        {
            try
            {
                UserModel storedCredentials = await _userRepository.GetByIdAsync(login.Id);
                byte[] salt = Convert.FromBase64String(storedCredentials.Salt);

                string computedHashedPassword = ComputeHashPassword(login.Password, salt);

                return computedHashedPassword.Equals(storedCredentials.Password) ? true : false;
            }
            catch (Exception ex) 
            {
                _logger.LogError(ex.Message);
                return false;
            }
        }

        /// <summary>
        /// Computes Hash + Salt for users'password
        /// </summary>
        /// <param name="clearTextPassword"></param>
        /// <param name="salt"></param>
        /// <returns> Hashed password </returns>
        private string ComputeHashPassword(string clearTextPassword, byte[] salt) 
        {
            string hashedPassword = Convert.ToBase64String(KeyDerivation.Pbkdf2(
                password: clearTextPassword,
                salt: salt,
                prf: KeyDerivationPrf.HMACSHA256,
                iterationCount: 100000,
                numBytesRequested: 256 / 8));
            return hashedPassword;
        }
    }
}
