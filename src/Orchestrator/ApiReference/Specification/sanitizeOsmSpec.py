# this script deletes the 5XX entries from yaml specification of the OSM API 
# the modified file has to be later conferted and used for the code generation 
# of the C# client for OSM

skip: bool = False
with open("OsmOpenApiSpec.yaml", "r") as f:
    lines = f.readlines()
with open("OsmOpenApiSpec.yaml", "w") as f:
    for line in lines:
        if skip:
            skip = False
            continue

        if "5XX" in line:
            skip = True
            continue

        f.write(line)
