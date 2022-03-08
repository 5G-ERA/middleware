-- This LUA script will query for all the policies inside redis with the attribute IsActive set to True. It will return to IsActive parameter and the Policy_Id.

redis.call('select', '3')

local ActivePoliciesArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    local val = redis.call('json.get', key, 'IsActive', 'Policy_Id')
    table.insert(ActivePoliciesArray, val);
end

return ActivePoliciesArray
