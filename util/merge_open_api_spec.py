from array import array
import sys
import json

# ResourcePlanner is not here because it is not accessible through gateway
files = ["./src/RedisInterface/RedisInterfaceSpec.json",
         "./src/TaskPlanner/TaskPlannerSpec.json"]
source_file = "./src/Orchestrator/OrchestratorSpec.json"
output_file = "./OpenAPISpec.json"

prefixes = {"RedisInterface": "Data",
            "TaskPlanner": "Task",
            "Orchestrator": "Orchestrate"}


def set_properties(template_info: dict) -> None:
    template_info["title"] = "5G-ERA Middleware"
    template_info["version"] = "0.1"


def add_routes(template: dict, routes: dict, api: str) -> None:
    for key in routes.keys():
        new_route_name = correct_route_name(key, api)
        template["paths"][new_route_name] = routes[key]


def add_models(template: dict, models: dict) -> None:
    for key in models.keys():
        if key not in template["components"]["schemas"]:
            template["components"]["schemas"][key] = models[key]


def correct_route_name(route: str, api: str) -> str:
    """
    Corrects the route path to match the gateway configuration

    Args:
        route: path to the endpoint.
        api: name of the api to adjust to.
    """
    prefix = prefixes[api]

    if "/api/v1/status/" in route and api == "Orchestrator":
        route = route.replace("/api/v1/status/", "/status/")

    if "/api/v1/" in route:
        route = route.replace("/api/v1/", "/" + prefix + "/")

    if "/api/" in route:
        route = route.replace("/api/", "/" + prefix + "/")

    return route


def get_api_name(file_name: str) -> str:
    return file_name.split("/")[2]


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
