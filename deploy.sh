#!/bin/sh -e
# Copyright 2021 Bartosz Bratuś
# See LICENSE file for licensing details.


if juju status | grep -q 'redis-operator'; then
    echo "Redis operator is already deployed. Updating application..."
    juju refresh redis-operator --path ./redis-operator_ubuntu-20.04-amd64.charm --resource redis-image=$VG_ERA_REDIS_IMAGE
else
    echo "Deploying redis operator..."
    juju deploy ./redis-operator_ubuntu-20.04-amd64.charm --resource redis-image=$VG_ERA_REDIS_IMAGE
fi


if juju status | grep -q 'cloud-storage-operator'; then
    echo "Cloud storage operator is already deployed. Updating application..."
    juju refresh cloud-storage-operator --path ./cloud-storage-operator_ubuntu-20.04-amd64.charm --resource cloud-storage-image=$VG_ERA_CLOUD_STORAGE_IMAGE
else
    echo "Deploying cloud storage operator..."
    juju deploy ./cloud-storage-operator_ubuntu-20.04-amd64.charm --resource cloud-storage-image=$VG_ERA_CLOUD_STORAGE_IMAGE
fi