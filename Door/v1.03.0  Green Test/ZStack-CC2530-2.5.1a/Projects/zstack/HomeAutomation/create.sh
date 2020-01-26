#!/bin/bash
#exit 1
PRO_NAME="IAS_Zone_Fire"
PRO_NAME_1="IAS_Zone_Fire"
PRO_NAME_2="IAS_ZONE_FIRE"

OLD_PRO_NAME="IAS_Zone_IR"
OLD_PRO_NAME_1="ias_Zone_IR"
OLD_PRO_NAME_2="IAS_ZONE_IR"


find Source CC2530DB -type f | xargs sed -i "s/${OLD_PRO_NAME}/${PRO_NAME}/g"
find Source CC2530DB -type f | xargs sed -i "s/${OLD_PRO_NAME_1}/${PRO_NAME_1}/g"
find Source CC2530DB -type f | xargs sed -i "s/${OLD_PRO_NAME_2}/${PRO_NAME_2}/g"

cd Source

mv OSAL_${OLD_PRO_NAME}.c OSAL_${PRO_NAME}.c
mv  zcl_${OLD_PRO_NAME_1}.c zcl_${PRO_NAME_1}.c
mv  zcl_${OLD_PRO_NAME_1}_data.c zcl_${PRO_NAME_1}_data.c
mv  zcl_${OLD_PRO_NAME_1}.h zcl_${PRO_NAME_1}.h

cd ..

cd CC2530DB

mv ${OLD_PRO_NAME}.ewd ${PRO_NAME}.ewd
mv ${OLD_PRO_NAME}.ewp ${PRO_NAME}.ewp 
mv ${OLD_PRO_NAME}.eww ${PRO_NAME}.eww

cd ..
