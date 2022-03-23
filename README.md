# Middleware

The repository that contains the middleware system applications of the 5G-ERA project.

## Setting up the development environment

The Middleware application is designed to be build around the Visual Studio 2022 installation utilizing the docker-compose support for the testing in the local development environment.

### System configuration

The following components have to be installed to successfully develop and test the applications in the DEV environment.

* **Visual Studio 2022** with the ASP.NET Web development package
![Visual StudioPackage](docs/img/VisualStudioPackage.png)
* **WSL2** (required for full docker support on Windows) for the instructions on how to install, refer to the official [guide](https://docs.microsoft.com/en-us/windows/wsl/install)
* **Docker Desktop** for running the build containers under docker compose. Follow the official [guide](https://docs.docker.com/desktop/windows/install/) for installation.

Additionally the Environment Variables have to be defined in the Operating system.
To do this go to:

`Menu Start -> View Advanced System Settings -> Environment Variables`

The new Environment Variables are added in using the button at the bottom:
![Add new ENV VAR](docs/img/AddEnvVariable.png)

Add 2 new Environment Variables if not defined already:

* **REDIS_HOSTNAME** - Address for the Redis server. It can be address of locally running server or the instance hosted by the 5G-ERA on AWS. The Redis needs to have installed additional modules: RedisGraph and RedisJSON
* **REDIS_PORT** - The port on which the Redis server is operating. The default is `6379`, for the AWS server it has different port

### Visual Studio Configuration

To properly run the solution from the Visual Studio specify the build configuration as in the image below:

![Visual Studio Run Config](docs/img/VSRunConfig.png)
The properties for the Build configuration should be set to `Debug` by default and the architecture for `Any CPU`. It is important to specify the project to run as `docker-compose` as it allows to seamless testing and debugging of the while solution without the need to change any configuration for the system.

Before the start, specify the project that you would like to test. To automatically display the Swagger UI for the desired project, click the Right Mouse Button on the `docker-compose` element in the Visual Studio Solution Explorer and click `Properties`.

![Open Properties of docker-compose project](docs/img/SetDockerComposeStartProp.png)

In the opened window navigate to the `Service Name` record in the table in the centre and select desired service to be started.
![Docker Compose Set startup project](docs/img/DockerComposeSetStartupProject.png)

With this property set, all the projects available will be started, but the one that will be opened will be the one selected in this menu.

## Launching the solution

After the environment is properly configured, use the launch button in the top of the Visual Studio under the search box. Click the button or use the `F5` key to launch the solution.

![Launch project button](docs/img/VSLaunchButton.png)

## Debug the solution

Debugging the solution is easy when launching the solution from the Visual Studio. Simply set the breakpoint using the `F9` key or by clicking in the vertical bar on the left of the code.

![Set breakpoint](docs/img/VSSetBreakpoint.png)
When the execution reaches the breakpoint, it will be stopped and you will see the code being stopped from there.

![Breakpoint hit](docs/img/VSBreakpointHit.png)

When the breakpoint is hit you can navigate with the `F10` key to step forward one line in code or `F11` to step inside the function that is to be executed.

## Handle changes to the API definition

The code contains the internal references to the current definition of the API using the OpenAPI 3.0 specification schema. Based on this specification the code is automatically generated to create the clients for easier access to the API endpoints from the code.

The API schema specification needs to be updated after each change to the endpoints definition. Each change of the return value, status code, adding new endpoints, removing existing or the definition of the models has to be followed with the appropriate procedure described, to make sure that all the APIs are working as expected.

### Update API definition procedure\

1. Launch the API in a standalone mode by selecting it as a startup project in the launch bar
![Visual Studio Set Startup Project](docs/img/VsApiStartupProject.png)
2. Navigate to the Health Controller definition in the opened browser window.
3. Execute endpoint `api/health/spec` to download the latest information about the API by pressing `Try it out button`
![Visual Studio Set Startup Project](docs/img/VsApiCallEndpoint.png)
4. Close browser window
5. Rebuild the whole solution to check for any errors across
6. Commit changes
