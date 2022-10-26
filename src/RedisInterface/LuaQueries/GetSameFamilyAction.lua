-- This LUA script will query for all actions that are of the same family to the provided one and have the same action tags.
-- This script is used for the replaning functions of the middleware.

-- launch script from ubuntu terminal: redis-cli --eval GetSameFamilyAction.lua , Navigation ROS2 FOXY 

-- Select index 10 of DB.
redis.call('select', '10')

-- Define method to check if variable is null
local function isempty(s)
  return s == nil or s == ''
end

local TagsParam = {};
local ActionCandidates = {};
local matches = redis.call('KEYS', '*')
local RobotROSVersion = "";
local RobotROSDistro = "";
local ReleasedDistros = {"FOXY","GALACTIC","MELODIC","NOETIC","LUNAR","KINETIC","INDIGO","HYDRO","FUERTE","GROOVY","ELECTRIC","HUMBLE","CRYSTAL","BOUNCY","DASHING","ARDENT" } -- In future read from another table of ros distros
local counter = 0


-- Check if element in array
local function has_value (tab, val)
    for index, value in ipairs(tab) do
        if value == val then
            return true
        end
    end

    return false
end

-- Get all params
for i = 1,1000 
do  
   if not isempty(ARGV[i]) then
        table.insert(tostring(ARGV[i]),TagsParam)
        if string.find(tostring(ARGV[i]), "ROS") then
            RobotROSVersion = ARGV[i]
        elseif has_value(ReleasedDistros,tostring(ARGV[i]) then 
            RobotROSDistro = ARGV[i]
        counter = counter +1
    else
        break
    end
end

for value,key in ipairs(matches) do

    local ActionId = redis.call('json.get', key, 'Id')
    local ActionFamily = redis.call('json.get', key, 'ActionFamily')
    local Tags = redis.call('json.get', key, 'Tags')
    local ActionAllObject = redis.call('json.get',key)

    -- Check if the action is from the same family, ros distro, ros version. TODO: extra tags.
    if (arg[1] == ActionFamily) then
        if string.find(Tags, "ROS2") then
            if RobotROSVersion == "ROS2" then 
                for i = 1, #ReleasedDistros do
                    if string.find(Tags, table[i]) then
                        table.insert(ActionCandidates,ActionId) 
                end
   
        else
            if RobotROSVersion == "ROS" then
                for i = 1, #ReleasedDistros do
                        if string.find(Tags, table[i]) then
                            table.insert(ActionCandidates,ActionId) 
                    end
end

return ActionCandidates

