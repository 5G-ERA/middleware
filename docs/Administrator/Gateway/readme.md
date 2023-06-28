# Dynamic Gateway configuration

The `Middleware` system offers the possibility to dynamically configure the `Gateway` in order to route the traffic through a websocket to a specific `Network Application` using the `YARP` reverse proxy functionalities. This allows enabling `RBAC` to the `Network Applications` and expose everything behind the `Gateway` using `SSL` termination. Moreover, this will also remove the need to enable the authentication on the `Network Application` level.

## How it works

The dynamic `Gateway` configuration is accomplished through standard `REST` request and `WebSockets`, in fact the whole mechanism is trigger through `RabbitMQ` queueing system for both creating the new route and deleting the route once the `Network Application` has finished conducting the task.
