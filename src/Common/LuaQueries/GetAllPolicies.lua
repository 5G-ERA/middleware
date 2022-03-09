-- This LUA script will query for all the policies inside redis returning the at parameter and the Policy_Id.

redis.call('select', '3')

local ActivePoliciesArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    local val = redis.call('json.get', key, 'Policy_Id','Description','PolicyName')
    table.insert(ActivePoliciesArray, val);
end

return ActivePoliciesArray
