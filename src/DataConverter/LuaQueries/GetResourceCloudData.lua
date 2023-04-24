--Query redis for resource All data based on resource name.
-- Select index 9 of DB.
redis.call('select', '9')

local resourceName = ARGV[1]
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do

    local resource = redis.call('json.get', key)
	 local resourceMatchName = redis.call('json.get', key, "$.Name")

	 if resourceMatchName == resourceName then
		return resource
	 end
end