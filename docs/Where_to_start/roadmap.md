# 5G-ERA Middleware

The repository contains the middleware system applications of the 5G-ERA project.
Visit the 5G ERA project website for more information.
https://5g-era.eu 

*The 5G-ERA project was developed by the following partners under the [consortium](https://5g-era.eu/consortium/) with funding of the European Union's Horizon 2020 Research and Innovation programme under grant agreement No. 101016681.*

For workshops demostrations and tutorials visit our [Youtube Channel](https://www.youtube.com/channel/UCFn5FI9OYLA9_jTwl2cwdFA/videos )

<p align="left">
  <img src="docs/img/logo.png" alt="MiddlewareArchitecture"/>
</p>

## What is 5G-ERA Middleware

5G-ERA Middleware is the Orchestration software responsible for linking vertical applications managed by ROS (Robot Operating System), a 5G infrastructure managed by Open Source MANO (OSM) and the cloud-native network applications ([NetApps](https://github.com/5G-ERA/Reference-NetApp)). It realizes the 5G intent-based networking by utilizing the cloud-native design. The core principles and functionality of the Middleware cover the lifecycle management, recovery and error handling of the applications, and integration of the semantic planning into the orchestration process.

## Where to start?
Please, for the very initial description, setup and configuration, [follow](https://github.com/5G-ERA/middleware/tree/main/docs/0_Where_To_Start) 

## Design and Architecture

The 5G-ERA Middleware is designed to connect three layers of application networks. First, it combines the ROS network on which the Robot specific application runs with the resource layer network. The Kubernetes network is managed by the OSM and the SDN Controllers that are part of the resource enablement network.
The Middleware mainly operates on the Kubernetes network layer, designed as a cloud-native application. The Middleware is built on the microservices architecture and consists of several components:
*	Gateway – It redirects the traffic across the Middleware system meaning rerouteing to the microservices within the system. It also handles the authentication and authorization process. 
* Action Planner – Integrating the semantic knowledge of the vertical into resource planning. It is part of the vertical level life cycle management implemented by the middleware. 
* Resource Planner – The resource Planner is responsible for assigning the placement example, on the cloud, Edge to the tasks. 
* Orchestrator – It orchestrates the process of the deployment of the resources. It is responsible for the vertical level lifecycle management of the deployed services 
* Redis Interface – Redis Interface allows the users to retrieve, insert and update data from/into the Redis-Server 

The image below presents the conceptual architecture of the 5G-ERA Middleware and the connection between the associated services. The 5G-ERA Middleware has been designed to run in the Cloud and the Edge devices. Therefore, it can be always accessible and provide the best quality of service to the Robot by placement in the closest possible location.
 
<p align="center">
  <img src="docs/img/Middleware_Architecture.png" alt="Middleware_Architecture"/ >
</p>

The 5G-ERA Middleware connects to the Redis Cluster, which allows sharing of semantic knowledge between all the instances of the 5G-ERA Middleware running in every Cloud and Edge device.

## Action Server

The communication between the 5G-ERA Middleware and the robot is conducted using a ROS Action Client, Action Server as well as Service Calls. Both ROS 1 and ROS2 versions are available for the Action-Server/client.

The action client and the robot itself are responsible for letting the middleware know via action server and microservices of the latest status of the action and NetApp. Two types of information are defined here: heartbeat and action status. The first one will be a request from the middleware with a specific frequency to measure the health of the deployed application/s that the currently executed action is using. The second one defines a high-level conceptualization of the status of the action. 

For a more in-detail description of the Action Server see [ACTION_SERVER_README.md](ACTION_SERVER_README.md)

## Middleware API specification

The 5G-ERA Middleware has an OpenAPI specification that can be checked under this [link](https://app.swaggerhub.com/apis/BARTOSZBRTATUS/5-g_era_middleware/0.1).

Try the API with the custom python library [link](https://github.com/5G-ERA/middleware/blob/main/src/Python_Interface_StandAlone/README.md).

## Using the 5G-ERA Middleware

To see how to deploy the 5G-ERA Middleware see the instructions in [ENVIRONMENT.md](ENVIRONMENT.md).

## Developing the 5G-ERA Middleware
To see how to deploy the 5G-ERA Middleware development enviroment, see the instructions in [ENVIRONMENT.md](ENVIRONMENT.md).

## License 
For the licensing information see [LICENSE](LICENSE).

## Middleware features & Bugfixing Roadmap:

Roadmap for continuos enhancement of the middleware.
<table style="width: 98%; margin-right: calc(2%);">
    <tbody>
        <tr>
            <td style="width: 19.9372%;"><br></td>
            <td style="width: 19.9372%;">Feature</td>
            <td style="width: 19.9686%;">Bug Fix</td>
            <td style="width: 19.9686%;">Date</td>
            <td style="width: 20.0000%;">Release</td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Robot/Cloud/Edge Data onboarding automatic middleware validation.</td>
            <td style="width: 19.9372%;">X</td>
            <td style="width: 19.9686%;"><br></td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                            <td style="width: 57.2585%;">October 2022</td>
                            <td style="width: 41.935%;"><span style="color: rgb(97, 189, 109);">No Delay</span></td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                            <td style="width: 18.9953%;">v0.16</td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Replanning Endpoint for robot tasks.&nbsp;</td>
            <td style="width: 19.9372%;">X</td>
            <td style="width: 19.9686%;"><br></td>
            <td style="width: 19.9686%;">
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                            <td style="width: 10.675%;">October 2022</td>
                            <td style="width: 8.6343%;"><span style="color: rgb(97, 189, 109);">No Delay</span></td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                            <td style="width: 18.9953%;">v0.16</td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Reference Robot Onboarding template</td>
            <td style="width: 19.9372%;">X</td>
            <td style="width: 19.9686%;"><br></td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                            <td style="width: 74.9996%;">October 2022</td>
                            <td style="width: 24.1939%;"><span style="color: rgb(97, 189, 109);">No Delay</span></td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                            <td style="width: 18.9953%;">v0.16</td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Bash script for automatic generation of onboarding files</td>
            <td style="width: 19.9372%;">X</td>
            <td style="width: 19.9686%;"><br></td>
            <td style="width: 19.9686%;"><br><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                            <td style="width: 13.1868%;">November 2022</td>
                            <td style="width: 6.1225%;"><span style="color: rgb(97, 189, 109);">No Delay</span></td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                            <td style="width: 18.9953%;">v0.17</td>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
    </tbody>
</table>

