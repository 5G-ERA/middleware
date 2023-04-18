using Ocelot.Middleware;
using System.Net;
using System.IdentityModel.Tokens.Jwt;
using Ocelot.Authorization;
using System.Linq;
using System.Security.Claims;

namespace Middleware.OcelotGateway.Services
{
    public static class OcelotAuthorizationMiddleware
    {
        public static async Task Authorize(HttpContext httpContext, Func<Task> next)
        {
            if (ValidateRole(httpContext))
                await next.Invoke();
            else
            {
                httpContext.Response.StatusCode = 403;
                httpContext.Items.SetError(new UnauthorizedError($"Authorisation Failed"));
            }
        }

        /*private static bool ValidateScope(HttpContext httpContext)
        {
            var downstreamRoute = httpContext.Items.DownstreamRoute();
            var listOfScopes = downstreamRoute.AuthenticationOptions.AllowedScopes;
            if (listOfScopes == null || listOfScopes.Count == 0) return true;
            var userClaimsPrincipals = httpContext.User.Claims.ToArray<Claim>();
            List<string> listOfClaimTypes = new List<string>();
            foreach (var userClaim in userClaimsPrincipals)
                listOfClaimTypes.Add(userClaim.Type);
            foreach (string scope in listOfScopes)
                if (!listOfClaimTypes.Contains(scope)) return false;
            return true;
        }*/
        private static bool ValidateRole(HttpContext ctx)
        {
            var downStreamRoute = ctx.Items.DownstreamRoute();
            if (downStreamRoute.AuthenticationOptions.AuthenticationProviderKey == null) return true;
            //This will get the claims of the users JWT
            Claim[] userClaims = ctx.User.Claims.ToArray();

            //This will get the required authorization claims of the route
            Dictionary<string, string> requiredAuthorizationClaims = downStreamRoute.RouteClaimsRequirement;

            //Getting the required claims for the route
            foreach (KeyValuePair<string, string> requiredAuthorizationClaim in requiredAuthorizationClaims)
            {
                if (ValidateIfStringIsRole(requiredAuthorizationClaim.Key))
                {
                    //Splitting the required claims as an or condition
                    foreach (var requiredClaimValue in requiredAuthorizationClaim.Value.Split(", "))
                    {
                        foreach (Claim userClaim in userClaims)
                        {
                            if (ValidateIfStringIsRole(userClaim.Type) && requiredClaimValue == userClaim.Value)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }
        private static bool ValidateIfStringIsRole(string role)
        {
            // The http://schemas.microsoft.com/ws/2008/06/identity/claims/role string is nesscary because Microsoft returns this as role claims in a JWT
            // And when directly adding this to the ocelot.json it will crash ocelot
            return role.Equals("http///schemas.microsoft.com/ws/2008/06/identity/claims/role") 
                || role.Equals("http://schemas.microsoft.com/ws/2008/06/identity/claims/role")
                || role.Equals("Role")
                || role.Equals("role");
        }
    }
}
