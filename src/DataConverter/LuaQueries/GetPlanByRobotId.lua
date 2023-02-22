-- This LUA script will query for the lastest plan given to a particular robot.

-- launch script from ubuntu terminal: redis-cli --eval GetSameFamilyAction.lua , Navigation ROS2 FOXY 

-- Select index 10 of DB.
redis.call('select', '13')

local plans = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do

    local PlanObject = redis.call('json.get',key)
    table.insert(plans,PlanObject) 

end

return plans

