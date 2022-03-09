-- This LUA script will query for all the policies inside redis returning only the ones with active policy check. Retur params are: policy name, id and description.

redis.call('select', '3')

local ActivePoliciesArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    local val = redis.call('json.get', key, 'Policy_Id','Description','PolicyName')
    table.insert(ActivePoliciesArray, val);
end

return ActivePoliciesArray
