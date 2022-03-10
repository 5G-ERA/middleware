-- This LUA script will return all active and not active policies of the system

redis.call('select', '3')

local ActivePoliciesArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    local val = redis.call('json.get', key, 'Policy_Id','Description','PolicyName')
    table.insert(ActivePoliciesArray, val);
end

return ActivePoliciesArray
