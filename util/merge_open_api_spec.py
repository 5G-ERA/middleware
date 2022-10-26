from array import array
import sys
import json
import os

dirname = os.path.dirname(__file__)

# ResourcePlanner is not here because it is not accessible through gateway
files = [os.path.join(dirname, "../src/RedisInterface/RedisInterfaceSpec.json"),
         os.path.join(dirname, "../src/TaskPlanner/TaskPlannerSpec.json"),
         os.path.join(dirname, "../src/OcelotGateway/OcelotGatewaySpec.json")]
source_file = os.path.join(
    dirname, "../src/Orchestrator/OrchestratorSpec.json")
output_file = os.path.join(dirname, "../OpenAPISpec.json")

prefixes = {"RedisInterface": "data",
            "TaskPlanner": "task",
            "Orchestrator": "orchestrate",
            "OcelotGateway": ""}


def set_properties(template_info: dict) -> None:
    template_info["title"] = "5G-ERA Middleware"
    template_info["version"] = "0.1"


def add_routes(template: dict, routes: dict, api: str) -> None:
    for key in routes.keys():
        new_route_name = correct_route_name(key, api)
        if not new_route_name:
            continue
        template["paths"][new_route_name] = routes[key]


def add_models(template: dict, models: dict) -> None:
    for key in models.keys():
        if key not in template["components"]["schemas"] and "File" not in key and "AggregateRouteConfig" not in key:
            template["components"]["schemas"][key] = models[key]


def correct_route_name(route: str, api: str) -> str:
    """
    Corrects the route path to match the gateway configuration
    If the route should be skipped, returns empty string.
    Args:
        route: path to the endpoint.
        api: name of the api to adjust to.
    """

    route = route.lower()
    if "/api/" not in route:
        return ""

    prefix = prefixes[api]

    if "/api/v1/status/" in route and api == "Orchestrator":
        route = route.replace("/api/v1/status/", "/status/")

    if "/api/v1/" in route and api == "OcelotGateway":
        route = route.replace("/api/v1/", "/")
    if "/api/v1/" in route:
        route = route.replace("/api/v1/", "/" + prefix + "/")
    if "/api/" in route:
        route = route.replace("/api/", "/" + prefix + "/")

    return route


def get_api_name(file_name: str) -> str:
    return file_name.split("/")[-2]


def main() -> None:
    """Entry point of the application, creates the template file"""
    with open(source_file, "r") as f:
        source_lines = f.read()

    template = dict(json.loads(source_lines))
    set_properties(template["info"])

    routes = template.pop("paths")
    template["paths"] = {}
    add_routes(template, routes, "Orchestrator")

    for file in files:
        with open(file, "r") as f:
            file_lines = f.read()
        file_json = dict(json.loads(file_lines))
        file_routes = file_json["paths"]
        file_models = file_json["components"]["schemas"]

        api_name = get_api_name(file)
        add_routes(template, file_routes, api_name)
        add_models(template, file_models)
        pass

    print(template)

    with open(output_file, "w") as f:
        f.write(json.dumps(template, indent=4))


if __name__ == "__main__":
    main()
